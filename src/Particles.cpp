// NukeVFX — simulation + rendering (see include/NukeVFX/Particles.h for the model).
#include <NukeVFX/Particles.h>
#include <API/Model/resdb.h>
#include <API/Model/Jobs.h>
#include <API/Model/Wind.h>
#include <API/Model/Physics.h>
#include <API/Model/Time.h>
#include <API/Model/Rand.h>
#include <API/Model/DebugDraw.h>
#include <API/Model/Light.h>          // FrameLights: glowing particles light the scene
#include <API/Model/BendVolumes.h>    // force fields bend foliage too (7.4)
#include <interface/Services.h>       // iWaterQuery: rain drops splash rings on water
#include <service/iWaterQuery.h>
#include <API/Model/MeshRenderer.h>   // Surface collision + editor-preview scene rays
#include <API/Model/Events.h>         // vfx.collision events (point/normal/atom/uv)
#include <API/Model/World.h>          // editor-preview World-mode collision scans the scene
#include <interface/AppInstance.h>
#include <render/irender.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <boost/thread/mutex.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cfloat>   // FLT_MAX (scene snapshot AABBs)

using namespace nuke;

namespace nuke {

// ---- ForceField registry -------------------------------------------------------------------
static boost::mutex gFFLock;
static std::vector<ForceField*> gFields;

ForceField::ForceField() : Component("ForceField") {}
void ForceField::Init(Atom* parent)
{
	atom = parent; transform = &parent->GetTransform();
	parent->components.push_back(this);
	boost::mutex::scoped_lock l(gFFLock);
	if (std::find(gFields.begin(), gFields.end(), this) == gFields.end()) gFields.push_back(this);
}
void ForceField::Destroy()
{
	boost::mutex::scoped_lock l(gFFLock);
	gFields.erase(std::remove(gFields.begin(), gFields.end(), this), gFields.end());
}
// Submits the field as an engine BendVolume (so foliage bends too) and draws the selected
// field's gizmo in the editor.
void ForceField::OnRender(iRender*, RenderPhase phase)
{
	if (phase != RenderPhase::Overlay || !transform) return;
	if (enabled)
	{
		const unsigned long long fr = Time::getSingleton()->frame;
		if (fr != bendSubmitFrame)   // OnRender fires per pass/camera — submit once
		{
			bendSubmitFrame = fr;
			Vector3 c = transform->globalPosition();
			BendVolume v;
			v.pos[0] = (float)c.x; v.pos[1] = (float)c.y; v.pos[2] = (float)c.z;
			v.radius = radius > 0.01f ? radius : 0.01f;
			v.falloff = falloff;
			switch (mode)
			{
				case 0:  v.mode = 1; v.strength = -strength; break;   // attract = inward radial
				case 1:  v.mode = 1; v.strength = strength;  break;   // repel
				case 2:  v.mode = 2; v.strength = strength;  break;   // vortex
				default: v.mode = 3; v.strength = strength;  break;   // turbulence
			}
			BendVolumes::Submit(v);
		}
	}
	AppInstance* app = AppInstance::GetSingleton();
	if (!app->isEditor() || app->selectedInHieararchy != atom) return;
	static const Color kModeCol[4] = { Color(0.4, 0.8, 1.0, 1.0),   // attract: blue
	                                   Color(1.0, 0.5, 0.3, 1.0),   // repel: orange
	                                   Color(0.7, 0.5, 1.0, 1.0),   // vortex: violet
	                                   Color(0.5, 1.0, 0.6, 1.0) }; // turbulence: green
	DebugDraw::WireSphere(transform->globalPosition(), radius, kModeCol[mode & 3]);
}

void ForceField::Update() {}
void ForceField::FixedUpdate() {}
void ForceField::Pause() {}
void ForceField::Reset() {}

// Snapshot of the enabled fields, taken once per frame so the sim jobs never touch the registry.
struct FFSnap { int mode; float center[3]; float radius; float strength; float falloff; };
static void SnapFields(std::vector<FFSnap>& out)
{
	boost::mutex::scoped_lock l(gFFLock);
	out.clear();
	for (ForceField* f : gFields)
	{
		if (!f || !f->enabled || !f->transform) continue;
		Vector3 c = f->transform->globalPosition();
		FFSnap s; s.mode = f->mode; s.center[0] = (float)c.x; s.center[1] = (float)c.y; s.center[2] = (float)c.z;
		s.radius = f->radius > 0.01f ? f->radius : 0.01f; s.strength = f->strength; s.falloff = f->falloff;
		out.push_back(s);
	}
}
// Summed acceleration from every snapshotted field at world point `p`.
static glm::vec3 FieldAccel(const std::vector<FFSnap>& fs, const glm::vec3& p, float seed, float t)
{
	glm::vec3 a(0);
	for (const FFSnap& f : fs)
	{
		glm::vec3 d = p - glm::vec3(f.center[0], f.center[1], f.center[2]);
		float dist = glm::length(d);
		if (dist > f.radius) continue;
		float w = 1.0f - f.falloff * (dist / f.radius);
		glm::vec3 dir = dist > 1e-5f ? d / dist : glm::vec3(0, 1, 0);
		switch (f.mode)
		{
			case 0: a -= dir * (f.strength * w); break;                        // attract
			case 1: a += dir * (f.strength * w); break;                        // repel
			case 2: a += glm::cross(glm::vec3(0, 1, 0), dir) * (f.strength * w); break;   // vortex around Y
			default:                                                           // turbulence
			{
				float s = f.strength * w;
				a += glm::vec3(sinf(p.y * 3.1f + t * 2.3f + seed * 17.f), sinf(p.z * 2.7f + t * 1.9f + seed * 11.f),
				               sinf(p.x * 3.3f + t * 2.1f + seed * 13.f)) * s;
				break;
			}
		}
	}
	return a;
}

// ---- curve/gradient evaluation (flattened key/stop arrays) --------------------------------
// Curve keys are stride-4: (t, value, inTangent, outTangent); tangents are slopes dv/dt and
// each segment is a cubic Hermite. Alpha is a coverage multiplier — always Clamp01 its result.
static float Clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
// Evaluates a stride-4 curve at t, or `def` when the curve is empty.
static float EvalCurve(const std::vector<float>& c, float t, float def)
{
	const size_t n = c.size() / 4;
	if (n == 0) return def;
	if (n == 1 || t <= c[0]) return c[1];
	if (t >= c[(n - 1) * 4]) return c[(n - 1) * 4 + 1];
	size_t i = 1;
	while (i < n && t > c[i * 4]) ++i;
	const float t0 = c[(i - 1) * 4], v0 = c[(i - 1) * 4 + 1];
	const float t1 = c[i * 4],       v1 = c[i * 4 + 1];
	const float h = t1 - t0;
	if (h < 1e-6f) return v1;
	const float m0 = c[(i - 1) * 4 + 3] * h;   // prev key's OUT tangent
	const float m1 = c[i * 4 + 2] * h;         // this key's IN tangent
	const float x = (t - t0) / h, x2 = x * x, x3 = x2 * x;
	return (2 * x3 - 3 * x2 + 1) * v0 + (x3 - 2 * x2 + x) * m0 + (-2 * x3 + 3 * x2) * v1 + (x3 - x2) * m1;
}
// Upgrades a legacy (t,v)-pair curve (size not a multiple of 4) to stride-4 keys with auto
// tangents, then sorts keys ascending by t — EvalCurve's segment search requires that.
static void MigrateCurve(std::vector<float>& c)
{
	if (!c.empty() && c.size() % 4 != 0 && c.size() % 2 == 0)
	{
		const size_t n = c.size() / 2;
		std::vector<float> out; out.reserve(n * 4);
		auto slope = [&](size_t k) -> float
		{
			size_t a = k == 0 ? 0 : k - 1, b = k == n - 1 ? n - 1 : k + 1;
			float dt = c[b * 2] - c[a * 2];
			return dt > 1e-6f ? (c[b * 2 + 1] - c[a * 2 + 1]) / dt : 0.f;
		};
		for (size_t k = 0; k < n; ++k)
		{
			float m = slope(k);
			out.push_back(c[k * 2]); out.push_back(c[k * 2 + 1]); out.push_back(m); out.push_back(m);
		}
		c.swap(out);
	}
	for (size_t k = 4; k + 3 < c.size(); k += 4)   // insertion sort by t (O(n) when already sorted)
		for (size_t j = k; j >= 4 && c[j] < c[j - 4]; j -= 4)
			for (int q = 0; q < 4; ++q) std::swap(c[j + q], c[j - 4 + q]);
}
// Evaluates a flattened (t,r,g,b) stop list at t into out[3]; leaves out untouched when empty.
static void EvalGradient(const std::vector<float>& g, float t, float out[3])
{
	const size_t n = g.size() / 4;
	if (n == 0) return;   // keep start color
	auto stop = [&](size_t i, float* rgb) { rgb[0] = g[i * 4 + 1]; rgb[1] = g[i * 4 + 2]; rgb[2] = g[i * 4 + 3]; };
	if (t <= g[0]) { stop(0, out); return; }
	for (size_t i = 1; i < n; ++i)
		if (t <= g[i * 4])
		{
			float t0 = g[(i - 1) * 4], t1 = g[i * 4];
			float f = (t1 - t0) > 1e-6f ? (t - t0) / (t1 - t0) : 0.f;
			float a[3], b[3]; stop(i - 1, a); stop(i, b);
			for (int k = 0; k < 3; ++k) out[k] = a[k] + (b[k] - a[k]) * f;
			return;
		}
	stop(n - 1, out);
}

// ---- built-in particle shapes -------------------------------------------------------------
// Procedural white RGBA texture (alpha = the shape) for the built-in sprite shapes, cached per
// process. Shape 0 (Quad) returns null = the white 1x1 fallback.
static Texture* ShapeTex(int shape)
{
	if (shape <= 0 || shape > 5) return nullptr;
	static Texture* cache[6] = {};
	if (cache[shape]) return cache[shape];
	const int S = 128;
	Texture* t = new Texture();
	t->width = S; t->height = S; t->format = Texture::FMT_RGBA8; t->mipCount = 1;
	snprintf(t->name, sizeof t->name, "vfx-shape-%d", shape);
	t->pixels.resize((size_t)S * S * 4);
	for (int y = 0; y < S; ++y)
		for (int x = 0; x < S; ++x)
		{
			float u = (x + 0.5f) / S * 2.f - 1.f, v = (y + 0.5f) / S * 2.f - 1.f;
			float rr = sqrtf(u * u + v * v);
			float a = 0.f;
			switch (shape)
			{
				case 1: a = 1.f - glm::smoothstep(0.55f, 1.0f, rr); break;                  // soft circle
				case 2: a = 1.f - glm::smoothstep(0.0f, 0.22f, fabsf(rr - 0.68f)); break;   // ring
				case 3:                                                                     // spark: hot core + 4-ray cross
				{
					float core = 1.f - glm::smoothstep(0.f, 0.35f, rr);
					float ax = fabsf(u), ay = fabsf(v);
					float rayH = (1.f - glm::smoothstep(0.f, 0.08f, ay)) * (1.f - glm::smoothstep(0.2f, 0.95f, ax));
					float rayV = (1.f - glm::smoothstep(0.f, 0.08f, ax)) * (1.f - glm::smoothstep(0.2f, 0.95f, ay));
					a = glm::clamp(std::max(core, std::max(rayH, rayV) * 0.9f), 0.f, 1.f);
					break;
				}
				case 4:                                                                     // 5-point star
				{
					float ang = atan2f(v, u);
					float lobe = cosf(ang * 5.f) * 0.5f + 0.5f;
					float edge = 0.35f + 0.55f * lobe;
					a = 1.f - glm::smoothstep(edge - 0.1f, edge + 0.05f, rr);
					break;
				}
				default:                                                                    // smoke: irregular soft blob
				{
					float n = sinf(u * 6.7f + v * 3.1f) * 0.5f + sinf(u * 2.9f - v * 7.3f + 1.7f) * 0.35f
					        + sinf((u + v) * 11.3f) * 0.15f;
					float edge = 0.75f + n * 0.18f;
					a = (1.f - glm::smoothstep(edge * 0.35f, edge, rr)) * 0.85f;
					break;
				}
			}
			unsigned char* px = &t->pixels[((size_t)y * S + x) * 4];
			px[0] = px[1] = px[2] = 255;
			px[3] = (unsigned char)(glm::clamp(a, 0.f, 1.f) * 255.f + 0.5f);
		}
	cache[shape] = t;
	return t;
}

// ---- exact ray vs meshes (collision Surface/editor-preview World) -------------------------
struct VfxRayHit { float t; glm::vec3 point, normal; float u, v; Atom* atom; };

// Möller–Trumbore over the mesh's unindexed triangles in LOCAL space (AABB slab early-out);
// fills point/normal/UV and returns whether anything was hit within maxT.
static bool RayMesh(Mesh* m, const glm::vec3& ro, const glm::vec3& rd, float maxT, VfxRayHit& out)
{
	if (!m || !m->vertexArray || m->numVerts < 3) return false;
	m->EnsureBounds();
	{
		float t0 = 0.f, t1 = maxT;
		for (int k = 0; k < 3; ++k)
		{
			const float o = (&ro.x)[k], d = (&rd.x)[k];
			if (fabsf(d) < 1e-8f) { if (o < m->aabbMin[k] || o > m->aabbMax[k]) return false; continue; }
			float ta = (m->aabbMin[k] - o) / d, tb = (m->aabbMax[k] - o) / d;
			if (ta > tb) std::swap(ta, tb);
			t0 = std::max(t0, ta); t1 = std::min(t1, tb);
			if (t0 > t1) return false;
		}
	}
	bool hit = false;
	const int tris = m->numVerts / 3;
	for (int tri = 0; tri < tris; ++tri)
	{
		const float* vp = m->vertexArray + (size_t)tri * 9;
		glm::vec3 a(vp[0], vp[1], vp[2]), b(vp[3], vp[4], vp[5]), c(vp[6], vp[7], vp[8]);
		glm::vec3 e1 = b - a, e2 = c - a;
		glm::vec3 pvv = glm::cross(rd, e2);
		float det = glm::dot(e1, pvv);
		if (fabsf(det) < 1e-9f) continue;
		float inv = 1.f / det;
		glm::vec3 tv = ro - a;
		float bu = glm::dot(tv, pvv) * inv;
		if (bu < 0.f || bu > 1.f) continue;
		glm::vec3 qv = glm::cross(tv, e1);
		float bv = glm::dot(rd, qv) * inv;
		if (bv < 0.f || bu + bv > 1.f) continue;
		float t = glm::dot(e2, qv) * inv;
		if (t < 1e-5f || t > maxT || (hit && t >= out.t)) continue;
		hit = true;
		out.t = t; out.point = ro + rd * t;
		glm::vec3 n = glm::cross(e1, e2);
		float l = glm::length(n);
		out.normal = l > 1e-9f ? n / l : glm::vec3(0, 1, 0);
		if (glm::dot(out.normal, rd) > 0.f) out.normal = -out.normal;   // face the incoming ray
		out.u = out.v = -1.f;
		if (m->uvArray)
		{
			const float* uv = m->uvArray + (size_t)tri * 6;
			float w0 = 1.f - bu - bv;
			out.u = uv[0] * w0 + uv[2] * bu + uv[4] * bv;
			out.v = uv[1] * w0 + uv[3] * bu + uv[5] * bv;
		}
	}
	return hit;
}

// Ray vs ONE atom's MeshRenderer, transform-aware (ray to local, hit back to world).
static bool RayAtomMesh(Atom* a, const glm::vec3& ro, const glm::vec3& rd, float maxT, VfxRayHit& out)
{
	MeshRenderer* mr = a ? a->GetComponent<MeshRenderer>() : nullptr;
	if (!mr || !mr->enabled || !mr->mesh) return false;
	Transform& t = a->GetTransform();
	Vector3 P = t.globalPosition(); Quaternion Q = t.globalRotation(); Vector3 S = t.globalScale();
	glm::mat4 w = glm::translate(glm::mat4(1.f), glm::vec3((float)P.x, (float)P.y, (float)P.z))
	            * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z))
	            * glm::scale(glm::mat4(1.f), glm::vec3((float)S.x, (float)S.y, (float)S.z));
	glm::mat4 inv = glm::inverse(w);
	glm::vec3 lro = glm::vec3(inv * glm::vec4(ro, 1));
	glm::vec3 lrd = glm::vec3(inv * glm::vec4(rd, 0));   // unnormalized on purpose: t stays in world units
	VfxRayHit lh;
	if (!RayMesh(mr->mesh, lro, lrd, maxT, lh)) return false;
	out.t = lh.t; out.u = lh.u; out.v = lh.v; out.atom = a;
	out.point = glm::vec3(w * glm::vec4(lh.point, 1));
	glm::vec3 wn = glm::vec3(glm::transpose(inv) * glm::vec4(lh.normal, 0));
	float l = glm::length(wn); out.normal = l > 1e-9f ? wn / l : glm::vec3(0, 1, 0);
	return true;
}

// Ray vs an atom SUBTREE (Surface mode: the referenced atom + its children = one surface).
static bool RayAtomSubtree(Atom* root, const glm::vec3& ro, const glm::vec3& rd, float maxT, VfxRayHit& out)
{
	if (!root) return false;
	bool hit = false; VfxRayHit h;
	std::vector<Atom*> stack{ root };
	while (!stack.empty())
	{
		Atom* a = stack.back(); stack.pop_back();
		if (!a) continue;
		if (RayAtomMesh(a, ro, rd, maxT, h)) { out = h; maxT = h.t; hit = true; }
		for (Atom* c : a->children) stack.push_back(c);
	}
	return hit;
}

// ---- editor-preview scene snapshot --------------------------------------------------------
// Outside play there are no physics bodies, so World collision raycasts scene meshes against
// this snapshot: cached inverse matrices + world AABBs, rebuilt once per frame (Time::frame).
struct SceneEntry { Atom* atom; Mesh* mesh; glm::mat4 w, inv; glm::vec3 mn, mx; };
static std::vector<SceneEntry> gScene;
static unsigned long long gSceneStamp = ~0ull;

// Rebuilds gScene for the current frame (no-op if already stamped).
static void SnapScene()
{
	const unsigned long long f = Time::getSingleton()->frame;
	if (f == gSceneStamp) return;
	gSceneStamp = f;
	gScene.clear();
	World* wl = AppInstance::GetSingleton()->currentWorld;
	if (!wl) return;
	std::vector<Atom*> stack;
	for (Atom* a : wl->GetHierarchy()) stack.push_back(a);
	while (!stack.empty())
	{
		Atom* a = stack.back(); stack.pop_back();
		if (!a) continue;
		for (Atom* c : a->children) stack.push_back(c);
		MeshRenderer* mr = a->GetComponent<MeshRenderer>();
		if (!mr || !mr->enabled || !mr->mesh || !mr->mesh->vertexArray || mr->mesh->numVerts < 3) continue;
		Transform& t = a->GetTransform();
		Vector3 P = t.globalPosition(); Quaternion Q = t.globalRotation(); Vector3 S = t.globalScale();
		SceneEntry e; e.atom = a; e.mesh = mr->mesh;
		e.w = glm::translate(glm::mat4(1.f), glm::vec3((float)P.x, (float)P.y, (float)P.z))
		    * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z))
		    * glm::scale(glm::mat4(1.f), glm::vec3((float)S.x, (float)S.y, (float)S.z));
		e.inv = glm::inverse(e.w);
		e.mesh->EnsureBounds();
		glm::vec3 mn(FLT_MAX), mx(-FLT_MAX);
		for (int ci = 0; ci < 8; ++ci)
		{
			glm::vec3 c(ci & 1 ? e.mesh->aabbMax[0] : e.mesh->aabbMin[0],
			            ci & 2 ? e.mesh->aabbMax[1] : e.mesh->aabbMin[1],
			            ci & 4 ? e.mesh->aabbMax[2] : e.mesh->aabbMin[2]);
			glm::vec3 wc = glm::vec3(e.w * glm::vec4(c, 1));
			mn = glm::min(mn, wc); mx = glm::max(mx, wc);
		}
		e.mn = mn; e.mx = mx;
		gScene.push_back(e);
	}
}

// Ray vs the frame's scene snapshot: world-AABB slab pre-test, then exact local triangles.
static bool RayScene(const glm::vec3& ro, const glm::vec3& rd, float maxT, VfxRayHit& out)
{
	SnapScene();
	bool hit = false;
	for (const SceneEntry& e : gScene)
	{
		float t0 = 0.f, t1 = maxT;
		bool miss = false;
		for (int k = 0; k < 3 && !miss; ++k)
		{
			const float o = (&ro.x)[k], d = (&rd.x)[k];
			if (fabsf(d) < 1e-8f) { if (o < (&e.mn.x)[k] || o > (&e.mx.x)[k]) miss = true; continue; }
			float ta = ((&e.mn.x)[k] - o) / d, tb = ((&e.mx.x)[k] - o) / d;
			if (ta > tb) std::swap(ta, tb);
			t0 = std::max(t0, ta); t1 = std::min(t1, tb);
			if (t0 > t1) miss = true;
		}
		if (miss) continue;
		glm::vec3 lro = glm::vec3(e.inv * glm::vec4(ro, 1));
		glm::vec3 lrd = glm::vec3(e.inv * glm::vec4(rd, 0));
		VfxRayHit lh;
		if (!RayMesh(e.mesh, lro, lrd, maxT, lh)) continue;
		hit = true; maxT = lh.t;
		out.t = lh.t; out.u = lh.u; out.v = lh.v; out.atom = e.atom;
		out.point = glm::vec3(e.w * glm::vec4(lh.point, 1));
		glm::vec3 wn = glm::vec3(glm::transpose(e.inv) * glm::vec4(lh.normal, 0));
		float l = glm::length(wn); out.normal = l > 1e-9f ? wn / l : glm::vec3(0, 1, 0);
	}
	return hit;
}

// ---- collision events ---------------------------------------------------------------------
static void JsonEsc(std::string& s, const std::string& in)
{
	for (char ch : in) { if (ch == '"' || ch == '\\') s += '\\'; s += ch; }
}
// Emits "vfx.collision" on the engine bus: emitter/hit atom names + ids, world point, normal
// and hit UV (physics hits carry u=v=-1).
static void EmitCollisionEvent(Atom* emitterAtom, const glm::vec3& pt, const glm::vec3& n, Atom* hitAtom, float u, float v)
{
	std::string j = "{\"emitter\":\"";
	if (emitterAtom) JsonEsc(j, emitterAtom->GetName());
	j += "\",\"hit\":\"";
	if (hitAtom) JsonEsc(j, hitAtom->GetName());
	char num[360];
	snprintf(num, sizeof num,
	         "\",\"emitterId\":%lu,\"hitId\":%lu,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,"
	         "\"nx\":%.3f,\"ny\":%.3f,\"nz\":%.3f,\"u\":%.4f,\"v\":%.4f}",
	         emitterAtom ? emitterAtom->id.id : 0ul, hitAtom ? hitAtom->id.id : 0ul,
	         pt.x, pt.y, pt.z, n.x, n.y, n.z, u, v);
	j += num;
	Events::EmitEngine("vfx.collision", j);
}

// ---- ParticleEmitter ----------------------------------------------------------------------

ParticleEmitter::ParticleEmitter() : Component("ParticleEmitter") {}

void ParticleEmitter::Init(Atom* parent)
{
	atom = parent; transform = &parent->GetTransform();
	parent->components.push_back(this);
}

void ParticleEmitter::Destroy()
{
	if (instBuf && instOwner) { instOwner->destroyInstanceBuffer(instBuf); instBuf = 0; instOwner = nullptr; }
	FreeRTMesh();
}

// Frees one RT mesh + material. The renderer's caches key on the Mesh pointer: invalidateMesh
// MUST happen before delete, or a rebuilt TLAS dereferences freed memory.
static void FreeOneRTMesh(Mesh*& m, Material*& mat, int& cap)
{
	if (m)
	{
		if (iRender* r = AppInstance::GetSingleton()->render) r->invalidateMesh(m);
		delete[] m->vertexArray; delete[] m->normalArray; delete[] m->uvArray; delete[] m->rtColorArray;
		m->vertexArray = m->normalArray = m->uvArray = m->rtColorArray = nullptr;
		delete m; m = nullptr;
	}
	delete mat; mat = nullptr; cap = 0;
}
void ParticleEmitter::FreeRTMesh()
{
	FreeOneRTMesh(rtMesh, rtMat, rtCap);
	FreeOneRTMesh(rtTrailMesh, rtTrailMat, rtTrailCap);
}

void ParticleEmitter::Update()
{
	Advance((float)Time::getSingleton()->gameDelta);   // scaled delta: freezes at timescale 0
}
void ParticleEmitter::FixedUpdate() {}
void ParticleEmitter::Pause() {}
void ParticleEmitter::Reset() { parts.clear(); trailHist.clear(); ageSec = 0.f; emitAccum = burstAccum = 0.f; hasLastPos = false; }

void ParticleEmitter::Play()  { playing = true; ageSec = 0.f; }
void ParticleEmitter::Stop()  { playing = false; }
void ParticleEmitter::Clear() { parts.clear(); trailHist.clear(); }
int  ParticleEmitter::AliveCount() { return (int)parts.size(); }

// Random in [a,b] off the engine's seeded stream (deterministic per world run).
static float Rnd(float a, float b) { return a + (float)Rand::Value("vfx") * (b - a); }

void ParticleEmitter::Burst(double count)
{
	Vector3 p = transform ? transform->globalPosition() : Vector3(0, 0, 0);
	BurstAt(p, count);
}

void ParticleEmitter::BurstAt(const Vector3& worldPos, double count)
{
	if (!transform) return;
	glm::vec3 base((float)worldPos.x, (float)worldPos.y, (float)worldPos.z);
	Quaternion Q = transform->globalRotation();
	glm::quat rot((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z);
	Mesh* emitMesh = nullptr;
	if (shape == 4 && !emitMeshGuid.empty()) emitMesh = ResDB::getSingleton()->GetMesh(emitMeshGuid);
	for (int i = 0; i < (int)count && (int)parts.size() < maxParticles; ++i)
	{
		glm::vec3 lp(0), dir(0, 1, 0);
		switch (shape)
		{
			case 0: dir = glm::normalize(glm::vec3(Rnd(-1, 1), Rnd(-1, 1), Rnd(-1, 1)) + glm::vec3(0, 1e-4f, 0)); break;
			case 1:   // sphere
			{
				glm::vec3 v(Rnd(-1, 1), Rnd(-1, 1), Rnd(-1, 1));
				float l = glm::length(v); if (l < 1e-4f) { v = glm::vec3(0, 1, 0); l = 1; }
				v /= l;
				lp = v * (fromShell ? shapeRadius : shapeRadius * powf((float)Rand::Value("vfx"), 1.f / 3.f));
				dir = v;
				break;
			}
			case 2:   // box
				lp = glm::vec3(Rnd(-1, 1) * (float)shapeExtents.x, Rnd(-1, 1) * (float)shapeExtents.y, Rnd(-1, 1) * (float)shapeExtents.z);
				dir = glm::vec3(0, 1, 0);
				break;
			case 3:   // cone from +Y
			{
				float ang = coneAngle * 0.0174533f * (fromShell ? 1.f : sqrtf((float)Rand::Value("vfx")));
				float az = Rnd(0, 6.28318f);
				dir = glm::vec3(sinf(ang) * cosf(az), cosf(ang), sinf(ang) * sinf(az));
				lp = glm::vec3(Rnd(-1, 1), 0, Rnd(-1, 1)) * shapeRadius;
				break;
			}
			case 4:   // mesh surface (uniform-ish: random triangle, barycentric point, its normal)
				if (emitMesh && emitMesh->numVerts >= 3)
				{
					int tri = (int)(Rand::Value("vfx") * (emitMesh->numVerts / 3 - 1));
					const float* vp = emitMesh->vertexArray;   // xyz per vert, unindexed
					glm::vec3 a(vp[(tri * 3 + 0) * 3], vp[(tri * 3 + 0) * 3 + 1], vp[(tri * 3 + 0) * 3 + 2]);
					glm::vec3 b(vp[(tri * 3 + 1) * 3], vp[(tri * 3 + 1) * 3 + 1], vp[(tri * 3 + 1) * 3 + 2]);
					glm::vec3 c(vp[(tri * 3 + 2) * 3], vp[(tri * 3 + 2) * 3 + 1], vp[(tri * 3 + 2) * 3 + 2]);
					float u = (float)Rand::Value("vfx"), v = (float)Rand::Value("vfx");
					if (u + v > 1.f) { u = 1.f - u; v = 1.f - v; }
					lp = a + (b - a) * u + (c - a) * v;
					glm::vec3 n = glm::cross(b - a, c - a);
					float l = glm::length(n); dir = l > 1e-6f ? n / l : glm::vec3(0, 1, 0);
				}
				break;
		}
		P pt{};
		glm::vec3 wp = localSpace ? lp : base + rot * lp;
		glm::vec3 wd = localSpace ? dir : rot * dir;
		pt.pos[0] = wp.x; pt.pos[1] = wp.y; pt.pos[2] = wp.z;
		float spd = Rnd(speedMin, speedMax);
		pt.vel[0] = wd.x * spd; pt.vel[1] = wd.y * spd; pt.vel[2] = wd.z * spd;
		if (inheritVelocity > 0.f && hasLastPos && !localSpace)
		{
			Vector3 cp = transform->globalPosition();
			double dt = Time::getSingleton()->gameDelta;
			if (dt > 1e-6)
			{
				pt.vel[0] += (float)((cp.x - lastPos[0]) / dt) * inheritVelocity;
				pt.vel[1] += (float)((cp.y - lastPos[1]) / dt) * inheritVelocity;
				pt.vel[2] += (float)((cp.z - lastPos[2]) / dt) * inheritVelocity;
			}
		}
		pt.maxLife = Rnd(lifeMin, lifeMax > lifeMin ? lifeMax : lifeMin);
		pt.life = pt.maxLife;
		pt.size = Rnd(sizeMin, sizeMax > sizeMin ? sizeMax : sizeMin);
		pt.rot = Rnd(0, 6.28318f);
		pt.rotVel = Rnd(-rotSpeed, rotSpeed) * 0.0174533f;
		pt.seed = (float)Rand::Value("vfx");
		parts.push_back(pt);
		trailHist.resize(parts.size() * kTrailCap * 3);
		float* h = trailHist.data() + (parts.size() - 1) * kTrailCap * 3;
		for (int k = 0; k < kTrailCap; ++k) { h[k * 3] = pt.pos[0]; h[k * 3 + 1] = pt.pos[1]; h[k * 3 + 2] = pt.pos[2]; }
	}
}

void ParticleEmitter::Advance(float dt)
{
	if (dt <= 0.f || !transform) return;
	MigrateCurve(sizeOverLife); MigrateCurve(alphaOverLife);   // legacy (t,v) pairs → tangent keys
	// ---- emission (game thread) ----
	ageSec += dt;
	const bool emitting = playing && (duration <= 0.f || ageSec <= duration);
	if (emitting)
	{
		if (rate > 0.f)
		{
			emitAccum += rate * dt;
			int n = (int)emitAccum;
			if (n > 0) { emitAccum -= n; Burst(n); }
		}
		if (burstCount > 0)
		{
			burstAccum += dt;
			while (burstAccum >= burstInterval) { burstAccum -= burstInterval; Burst(burstCount); }
		}
	}
	Vector3 cp = transform->globalPosition();
	lastPos[0] = cp.x; lastPos[1] = cp.y; lastPos[2] = cp.z; hasLastPos = true;

	// A live maxParticles reduction applies immediately (overflow dies now).
	if ((int)parts.size() > maxParticles && maxParticles > 0)
	{
		parts.resize(maxParticles);
		trailHist.resize((size_t)maxParticles * kTrailCap * 3);
	}

	if (parts.empty()) return;

	// ---- integrate (parallel; PLAIN DATA only — the Jobs rule) ----
	static std::vector<FFSnap> fs; SnapFields(fs);
	const float g = gravity * 9.81f;
	const float windI = windInfluence;
	const float dragK = drag;
	const float t = ageSec;
	// Wind::Sample locks the zone registry — sample ONCE and reuse for the whole batch.
	Vector3 wv = windI > 0.f ? Wind::Sample(Vector3(cp.x, cp.y, cp.z)) : Vector3(0, 0, 0);
	glm::vec3 wind((float)wv.x, (float)wv.y, (float)wv.z);
	std::vector<P>& ps = parts;
	Jobs::ParallelFor(0, (int)ps.size(), 0, [&](int i)
	{
		P& p = ps[i];
		glm::vec3 pos(p.pos[0], p.pos[1], p.pos[2]);
		glm::vec3 vel(p.vel[0], p.vel[1], p.vel[2]);
		glm::vec3 acc(0, -g, 0);
		acc += FieldAccel(fs, pos, p.seed, t);
		if (windI > 0.f) acc += (wind - vel) * windI;          // relax toward the wind vector
		if (dragK > 0.f) acc -= vel * dragK;
		vel += acc * dt;
		pos += vel * dt;
		p.rot += p.rotVel * dt;
		p.life -= dt;
		p.pos[0] = pos.x; p.pos[1] = pos.y; p.pos[2] = pos.z;
		p.vel[0] = vel.x; p.vel[1] = vel.y; p.vel[2] = vel.z;
	});

	// ---- trail history shift (newest point at the block tail) ----
	if ((trailEnabled || renderMode == 2) && trailHist.size() == parts.size() * (size_t)kTrailCap * 3)
		Jobs::ParallelFor(0, (int)parts.size(), 0, [&](int i)
		{
			float* h = trailHist.data() + (size_t)i * kTrailCap * 3;
			memmove(h, h + 3, (kTrailCap - 1) * 3 * sizeof(float));
			h[(kTrailCap - 1) * 3]     = parts[i].pos[0];
			h[(kTrailCap - 1) * 3 + 1] = parts[i].pos[1];
			h[(kTrailCap - 1) * 3 + 2] = parts[i].pos[2];
		});

	// ---- collision + expiry (game thread: physics + sub-emitter are not job-safe) ----
	glm::mat4 l2w(1.0f);
	if (localSpace)
	{
		Quaternion Q = transform->globalRotation();
		l2w = glm::translate(glm::mat4(1.f), glm::vec3((float)cp.x, (float)cp.y, (float)cp.z)) * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z));
	}
	// Ray budget: particles take turns round-robin over K frames and the ray reach is stretched
	// by K, so skipped frames cannot tunnel.
	AppInstance* colApp = AppInstance::GetSingleton();
	const bool editorPreview = colApp->isEditor() && colApp->playState == 0;
	iWaterQuery* wq = waterContact ? GetService<iWaterQuery>() : nullptr;   // rain -> rings
	int eventsDone = 0;
	const int alive = (int)parts.size();
	const int budget = collisionBudget <= 0 ? alive : collisionBudget;
	const int K = (alive > budget && budget > 0) ? (alive + budget - 1) / budget : 1;
	rayCursor = K > 1 ? (rayCursor + 1) % K : 0;
	for (size_t i = 0; i < parts.size(); )
	{
		P& p = parts[i];
		bool dead = p.life <= 0.f, collided = false;
		glm::vec3 hitP(0);
		if (!dead && collision != 0)
		{
			glm::vec3 wpos(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wpos = glm::vec3(l2w * glm::vec4(wpos, 1));
			glm::vec3 v(p.vel[0], p.vel[1], p.vel[2]);
			float spd = glm::length(v);
			const bool due = K == 1 || (int)(i % (size_t)K) == rayCursor;   // this particle's turn
			if (spd > 1e-4f && due)
			{
				glm::vec3 rdir = v / spd;
				const float reach = spd * dt * 1.5f * K + p.size * 0.5f;   // cover the skipped frames
				bool hit = false; glm::vec3 hp(0), hn(0, 1, 0); float hu = -1.f, hv = -1.f; Atom* hitAtom = nullptr;
				if (collision == 2 && collisionSurface)   // Surface: ONLY the referenced atom's meshes
				{
					VfxRayHit rh;
					if (RayAtomSubtree(collisionSurface, wpos, rdir, reach, rh))
					{ hit = true; hp = rh.point; hn = rh.normal; hu = rh.u; hv = rh.v; hitAtom = rh.atom; }
				}
				else                                       // World (a Surface with no ref falls back here)
				{
					if (!editorPreview)
					{
						if (Physics::Raycast(Vector3(wpos.x, wpos.y, wpos.z), Vector3(rdir.x, rdir.y, rdir.z), reach))
						{
							hit = true;
							Vector3 hpp = Physics::HitPoint(); hp = glm::vec3((float)hpp.x, (float)hpp.y, (float)hpp.z);
							Vector3 hnn = Physics::HitNormal(); hn = glm::vec3((float)hnn.x, (float)hnn.y, (float)hnn.z);
							hitAtom = Physics::HitAtom();
						}
					}
					else
					{
						VfxRayHit rh;
						if (RayScene(wpos, rdir, reach, rh))
						{ hit = true; hp = rh.point; hn = rh.normal; hu = rh.u; hv = rh.v; hitAtom = rh.atom; }
					}
				}
				if (hit)
				{
					collided = true; hitP = hp;
					// bounce the normal component, dampen the tangential slide
					glm::vec3 refl = v - 2.f * glm::dot(v, hn) * hn;
					glm::vec3 rn = hn * glm::dot(refl, hn);
					glm::vec3 rt = refl - rn;
					refl = rn * bounce + rt * (1.f - collideDampen);
					p.vel[0] = refl.x; p.vel[1] = refl.y; p.vel[2] = refl.z;
					glm::vec3 np = hp + hn * 0.01f;
					if (localSpace) np = glm::vec3(glm::inverse(l2w) * glm::vec4(np, 1));
					p.pos[0] = np.x; p.pos[1] = np.y; p.pos[2] = np.z;
					if (collisionEvents && eventsDone < 64)
					{ ++eventsDone; EmitCollisionEvent(atom, hp, hn, hitAtom, hu, hv); }
				}
			}
			if (collided && dieOnCollision) dead = true;
			if (collided && subOnCollision && subEmitter) subBurstPts.insert(subBurstPts.end(), { hitP.x, hitP.y, hitP.z });
		}
		if (!dead && wq)
		{
			glm::vec3 wp2(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wp2 = glm::vec3(l2w * glm::vec4(wp2, 1));
			const double lvl = wq->HeightAt(wp2.x, wp2.z);
			if (lvl > -1e8 && wp2.y <= (float)lvl)
			{
				const float hp3[3] = { wp2.x, (float)lvl, wp2.z };
				wq->Splash(hp3, p.size * 0.9f + 0.12f, 0.15f + fabsf(p.vel[1]) * 0.05f);
				if (dieOnWater) dead = true;
				else p.vel[1] = fabsf(p.vel[1]) * 0.25f;   // damped pop back out
			}
		}
		if (dead)
		{
			if (!subOnCollision && subEmitter)
			{
				// death burst point in WORLD coords (p.pos is local when localSpace is on)
				glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
				if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
				subBurstPts.insert(subBurstPts.end(), { wp.x, wp.y, wp.z });
			}
			const size_t bs = (size_t)kTrailCap * 3;
			if (i + 1 < parts.size())
				memcpy(trailHist.data() + i * bs, trailHist.data() + (parts.size() - 1) * bs, bs * sizeof(float));
			parts[i] = parts.back(); parts.pop_back();
			trailHist.resize(parts.size() * bs);
			continue;
		}
		++i;
	}

	// ---- sub-emitter bursts (game thread) ----
	if (!subBurstPts.empty() && subEmitter)
	{
		if (ParticleEmitter* se = subEmitter->GetComponent<ParticleEmitter>())
			for (size_t k = 0; k + 2 < subBurstPts.size(); k += 3)
				se->BurstAt(Vector3(subBurstPts[k], subBurstPts[k + 1], subBurstPts[k + 2]), subEmitterCount);
		subBurstPts.clear();
	}

	SubmitLights();   // one-frame submissions, consumed by World::Render's light pack
}

// ---- rendering ----------------------------------------------------------------------------

void ParticleEmitter::OnRender(iRender* r, RenderPhase phase)
{
	// Selected-emitter gizmo (editor): the emission shape as wire geometry.
	if (phase == RenderPhase::Overlay && transform)
	{
		AppInstance* gApp = AppInstance::GetSingleton();
		if (gApp->isEditor() && gApp->selectedInHieararchy == atom)
		{
			const Color gc(1.0, 0.72, 0.25, 1.0);   // emitter orange
			Vector3 gp = transform->globalPosition();
			Quaternion gq = transform->globalRotation();
			glm::quat grot((float)gq.w, (float)gq.x, (float)gq.y, (float)gq.z);
			glm::vec3 gup = grot * glm::vec3(0, 1, 0);
			switch (shape)
			{
				case 0: DebugDraw::WireSphere(gp, 0.12, gc); break;                                   // point
				case 1: DebugDraw::WireSphere(gp, shapeRadius, gc); break;                            // sphere
				case 2: DebugDraw::WireBox(gp, shapeExtents, gq, gc); break;                          // box
				case 3:                                                                                // cone
					DebugDraw::WireCone(gp, Vector3(gup.x, gup.y, gup.z), coneAngle,
					                    std::max(1.0f, (speedMin + speedMax) * 0.35f), gc);
					if (shapeRadius > 0.01f)
						DebugDraw::WireCircle(gp, Vector3(gup.x, gup.y, gup.z), shapeRadius, gc);     // spawn disc
					break;
				default: DebugDraw::WireSphere(gp, 0.25, gc); break;                                  // mesh surface marker
			}
		}
	}
	// RT gather: must run between beginRTScene/buildRTScene.
	if (phase == RenderPhase::RTScene)
	{
		if (r && r->rtAvailable() && (inReflections || castShadows) && transform)
			BuildRTQuads(r);
		return;
	}
	if (phase != RenderPhase::Transparent || !r) return;
	// Editor preview: while not playing the sim advances from the render hook instead of Update.
	AppInstance* app = AppInstance::GetSingleton();
	if (app->playState == 0) Advance((float)Time::getSingleton()->delta);
	Draw(r);
}

// Re-resolves the asset caches whenever their guid prop changed; a failed resolve stores no
// guid, so a not-yet-loaded asset is retried next frame.
void ParticleEmitter::ResolveAssets()
{
	if (textureGuid != texGuidRes)
	{
		texCache = textureGuid.empty() ? nullptr : ResDB::getSingleton()->GetTexture(textureGuid);
		if (textureGuid.empty() || texCache) texGuidRes = textureGuid;
	}
	if (meshGuid != meshGuidRes)
	{
		meshCache = meshGuid.empty() ? nullptr : ResDB::getSingleton()->GetMesh(meshGuid);
		if (meshGuid.empty() || meshCache) meshGuidRes = meshGuid;
	}
	if (materialGuid != matGuidRes)
	{
		matCache = materialGuid.empty() ? nullptr : ResDB::getSingleton()->GetMaterial(materialGuid);
		if (materialGuid.empty() || matCache) matGuidRes = materialGuid;
	}
	if (trailTextureGuid != trailTexGuidRes)
	{
		trailTexCache = trailTextureGuid.empty() ? nullptr : ResDB::getSingleton()->GetTexture(trailTextureGuid);
		if (trailTextureGuid.empty() || trailTexCache) trailTexGuidRes = trailTextureGuid;
	}
}

void ParticleEmitter::Draw(iRender* r)
{
	if (parts.empty() || !transform) return;
	ResolveAssets();

	// Legacy renderMode 2 = billboard + trail.
	const bool wantTrail = trailEnabled || renderMode == 2;
	const int  baseMode  = renderMode == 2 ? 0 : renderMode;

	// Camera basis: view is row-major, so right/up/fwd read out of its columns.
	float view[16], proj[16];
	r->getViewProj(view, proj);
	glm::vec3 right(view[0], view[4], view[8]);
	glm::vec3 up(view[1], view[5], view[9]);

	glm::mat4 l2w(1.0f);
	if (localSpace)
	{
		Vector3 cp = transform->globalPosition(); Quaternion Q = transform->globalRotation();
		l2w = glm::translate(glm::mat4(1.f), glm::vec3((float)cp.x, (float)cp.y, (float)cp.z)) * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z));
	}

	// MESH mode: pack instance records, one instanced draw; falls through when a trail is on.
	bool meshDrawn = false;
	if (baseMode == 3 && meshCache)
	{
		if (instBuf && instOwner != r) { instBuf = 0; instOwner = nullptr; }
		if (!instBuf) { instBuf = r->createInstanceBuffer(); instOwner = r; }
		if (instBuf)
		{
		static std::vector<NukeInstanceData> recs; recs.clear(); recs.reserve(parts.size());
		for (const P& p : parts)
		{
			float lt = 1.f - p.life / p.maxLife;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
			glm::mat4 w = glm::translate(glm::mat4(1.f), wp)
			            * glm::mat4_cast(glm::angleAxis(p.rot, glm::normalize(glm::vec3(0.3f, 1, 0.2f))))
			            * glm::scale(glm::mat4(1.f), glm::vec3(sz));
			NukeInstanceData rec;
			for (int k = 0; k < 4; ++k) rec.row0[k] = w[k][0];
			for (int k = 0; k < 4; ++k) rec.row1[k] = w[k][1];
			for (int k = 0; k < 4; ++k) rec.row2[k] = w[k][2];
			float rgb[3] = { (float)startColor.r, (float)startColor.g, (float)startColor.b };
			EvalGradient(colorGradient, lt, rgb);
			const float gEffM = glow * EvalCurve(glowOverLife, lt, 1.f);
			if (gEffM > 0.f) { const float g = 1.f + gEffM; rgb[0] *= g; rgb[1] *= g; rgb[2] *= g; }   // HDR boost -> bloom
			rec.color[0] = rgb[0]; rec.color[1] = rgb[1]; rec.color[2] = rgb[2];
			rec.color[3] = (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f));
			rec.custom[0] = lt; rec.custom[1] = p.seed;
			recs.push_back(rec);
		}
		r->updateInstanceBuffer(instBuf, recs.data(), (int)recs.size());
		r->renderObjectInstanced(meshCache, matCache, instBuf, 0, (int)recs.size());
		meshDrawn = true;
		}
	}
	if (baseMode == 3 && !wantTrail) return;
	(void)meshDrawn;

	// SPRITE modes: build quads (9 floats/vert, 6 verts/quad) into one run per emitter.
	static std::vector<float> verts; verts.clear();
	static std::vector<int> order; order.clear();
	const int n = (int)parts.size();
	for (int i = 0; i < n; ++i) order.push_back(i);
	if (sortParticles)
	{
		glm::vec3 camF(view[2], view[6], view[10]);
		std::sort(order.begin(), order.end(), [&](int a, int b) {
			const P& A = parts[a], &B = parts[b];
			float da = A.pos[0] * camF.x + A.pos[1] * camF.y + A.pos[2] * camF.z;
			float db = B.pos[0] * camF.x + B.pos[1] * camF.y + B.pos[2] * camF.z;
			return da > db;   // far first
		});
	}
	auto quad = [&](const glm::vec3& c, const glm::vec3& rv, const glm::vec3& uv2, const float col[4])
	{
		const glm::vec3 v0 = c - rv - uv2, v1 = c + rv - uv2, v2 = c + rv + uv2, v3 = c - rv + uv2;
		const float q[6][5] = { { v0.x, v0.y, v0.z, 0, 1 }, { v1.x, v1.y, v1.z, 1, 1 }, { v2.x, v2.y, v2.z, 1, 0 },
		                        { v0.x, v0.y, v0.z, 0, 1 }, { v2.x, v2.y, v2.z, 1, 0 }, { v3.x, v3.y, v3.z, 0, 0 } };
		for (int k = 0; k < 6; ++k)
		{
			verts.insert(verts.end(), { q[k][0], q[k][1], q[k][2], q[k][3], q[k][4], col[0], col[1], col[2], col[3] });
		}
	};
	if (baseMode != 3)
		for (int oi : order)
		{
			const P& p = parts[oi];
			float lt = 1.f - p.life / p.maxLife;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			float col[4] = { (float)startColor.r, (float)startColor.g, (float)startColor.b,
			                 (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f)) };
			EvalGradient(colorGradient, lt, col);
			const float gEffS = glow * EvalCurve(glowOverLife, lt, 1.f);   // emission breathes over life
			if (gEffS > 0.f) { const float g = 1.f + gEffS; col[0] *= g; col[1] *= g; col[2] *= g; }   // HDR boost -> bloom
			if (blend == 1) { col[0] *= col[3]; col[1] *= col[3]; col[2] *= col[3]; }   // additive: premodulate
			glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
			if (baseMode == 1)   // velocity-stretched
			{
				glm::vec3 v(p.vel[0], p.vel[1], p.vel[2]);
				float spd = glm::length(v);
				glm::vec3 axis = spd > 1e-4f ? v / spd : up;
				glm::vec3 side = glm::normalize(glm::cross(axis, glm::vec3(view[2], view[6], view[10])) + glm::vec3(1e-5f));
				quad(wp, side * (sz * 0.5f), axis * (sz * 0.5f + spd * stretch), col);
			}
			else                 // camera-facing billboard (rotated)
			{
				float cr = cosf(p.rot), sr = sinf(p.rot);
				glm::vec3 rv = (right * cr + up * sr) * (sz * 0.5f);
				glm::vec3 uv2 = (up * cr - right * sr) * (sz * 0.5f);
				quad(wp, rv, uv2, col);
			}
		}
	// TRAIL ribbon: camera-facing strip over the history. v runs continuously head(0) -> tail(1)
	// so the trail texture stretches over the WHOLE ribbon, not once per segment.
	static std::vector<float> trailV; trailV.clear();
	if (wantTrail && trailHist.size() == parts.size() * (size_t)kTrailCap * 3)
	{
		glm::vec3 camF(view[2], view[6], view[10]);
		const int segs = trailSegments < 2 ? 2 : (trailSegments > kTrailCap ? kTrailCap : trailSegments);
		auto pv = [&](const glm::vec3& v, float u, float vv, const float* c)
		{ trailV.insert(trailV.end(), { v.x, v.y, v.z, u, vv, c[0], c[1], c[2], c[3] }); };
		for (int oi : order)
		{
			const P& p = parts[oi];
			float lt = 1.f - p.life / p.maxLife;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			float col[4] = { (float)startColor.r, (float)startColor.g, (float)startColor.b,
			                 (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f)) };
			EvalGradient(colorGradient, lt, col);
			const float gEffT = glow * EvalCurve(glowOverLife, lt, 1.f);
			if (gEffT > 0.f) { const float g = 1.f + gEffT; col[0] *= g; col[1] *= g; col[2] *= g; }   // HDR boost -> bloom
			const float* h = trailHist.data() + (size_t)oi * kTrailCap * 3;
			const float headW = std::max(0.001f, sz * trailWidth * 0.5f);   // half-width at the head
			for (int sgi = kTrailCap - segs; sgi < kTrailCap - 1; ++sgi)
			{
				glm::vec3 a(h[sgi * 3], h[sgi * 3 + 1], h[sgi * 3 + 2]);                       // older
				glm::vec3 b(h[(sgi + 1) * 3], h[(sgi + 1) * 3 + 1], h[(sgi + 1) * 3 + 2]);     // newer
				if (localSpace) { a = glm::vec3(l2w * glm::vec4(a, 1)); b = glm::vec3(l2w * glm::vec4(b, 1)); }
				glm::vec3 d = b - a;
				if (glm::dot(d, d) < 1e-10f) continue;
				glm::vec3 side = glm::normalize(glm::cross(glm::normalize(d), camF) + glm::vec3(1e-6f));
				// along = 0 at the head (newest point), 1 at the tail (oldest drawn point)
				const float alongA = (float)(kTrailCap - 1 - sgi) / (float)(segs - 1);
				const float alongB = (float)(kTrailCap - 2 - sgi) / (float)(segs - 1);
				const float wA = headW * (1.f - trailTaper * alongA);
				const float wB = headW * (1.f - trailTaper * alongB);
				float c0[4] = { col[0], col[1], col[2], col[3] * (1.f - trailFade * alongA) };
				float c1[4] = { col[0], col[1], col[2], col[3] * (1.f - trailFade * alongB) };
				if (blend == 1)
				{
					for (int k = 0; k < 3; ++k) { c0[k] *= c0[3]; c1[k] *= c1[3]; }
				}
				glm::vec3 s0 = side * wA, s1 = side * wB;
				glm::vec3 v0 = a - s0, v1 = a + s0, v2 = b + s1, v3 = b - s1;
				pv(v0, 0, alongA, c0); pv(v1, 1, alongA, c0); pv(v2, 1, alongB, c1);
				pv(v0, 0, alongA, c0); pv(v2, 1, alongB, c1); pv(v3, 0, alongB, c1);
			}
		}
	}
	if (!verts.empty() || !trailV.empty())
	{
		if (softFade > 0.f) r->setSpriteSoftDepth(softFade);
		// trail first: ribbons sit BEHIND their particles
		if (!trailV.empty())
			r->drawSpriteRun(trailTexCache, trailV.data(), (int)trailV.size() / 9);
		if (!verts.empty())
		{
			// built-in procedural shape when no texture asset is assigned (Quad = plain white)
			Texture* baseTex = texCache ? texCache : ShapeTex(spriteShape);
			r->drawSpriteRun(baseTex, verts.data(), (int)verts.size() / 9);
		}
		if (softFade > 0.f) r->setSpriteSoftDepth(0.f);   // restore: other sprite users stay hard
	}
}

// Refreshes this frame's RT geometry (sprite quads, trail ribbons, mesh-mode instances) and
// registers it in the TLAS. Faded particles (alpha < 0.02) drop out.
void ParticleEmitter::BuildRTQuads(iRender* r)
{
	if (parts.empty()) return;   // no addRTInstance -> not in the TLAS this frame
	ResolveAssets();
	const bool wantTrail = trailEnabled || renderMode == 2;
	const int  baseMode  = renderMode == 2 ? 0 : renderMode;

	float view[16], proj[16];
	r->getViewProj(view, proj);
	glm::vec3 right(view[0], view[4], view[8]);
	glm::vec3 up(view[1], view[5], view[9]);
	glm::vec3 camF(view[2], view[6], view[10]);
	glm::mat4 l2w(1.0f);
	if (localSpace)
	{
		Vector3 cp = transform->globalPosition(); Quaternion Q = transform->globalRotation();
		l2w = glm::translate(glm::mat4(1.f), glm::vec3((float)cp.x, (float)cp.y, (float)cp.z))
		    * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z));
	}
	const int   cap = maxParticles < 1 ? 1 : maxParticles;
	const int   n   = (int)parts.size() < cap ? (int)parts.size() : cap;
	float alphaSum = 0.f; int alphaN = 0;

	// ---- MESH mode: one TLAS instance per particle over the asset mesh (cached BLAS) ----
	if (baseMode == 3 && meshCache)
	{
		for (int i = 0; i < n; ++i)
		{
			const P& p = parts[i];
			float lt = 1.f - p.life / p.maxLife;
			float a  = (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f));
			if (a < 0.02f) continue;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
			glm::quat q = glm::angleAxis(p.rot, glm::normalize(glm::vec3(0.3f, 1, 0.2f)));
			float pos[3]   = { wp.x, wp.y, wp.z };
			float quat[4]  = { q.x, q.y, q.z, q.w };
			float scale[3] = { sz, sz, sz };
			r->addRTInstance(meshCache, matCache, pos, quat, scale, inReflections, castShadows);
		}
	}

	// ---- SPRITE modes: the per-frame quad mesh (billboard or velocity-stretched) ----
	if (baseMode != 3)
	{
		if (!rtMesh || rtCap != cap)
		{
			FreeOneRTMesh(rtMesh, rtMat, rtCap);
			rtMesh = new Mesh();
			rtCap  = cap;
			strncpy(rtMesh->name, "particles_rt", sizeof(rtMesh->name) - 1);
			rtMesh->numVerts     = cap * 6;
			rtMesh->vertexArray  = new float[(size_t)cap * 18]();   // zeros = degenerate quads
			rtMesh->normalArray  = new float[(size_t)cap * 18];
			rtMesh->uvArray      = new float[(size_t)cap * 12];
			rtMesh->rtColorArray = new float[(size_t)cap * 24]();   // float4 x 6 verts per quad
			rtMesh->rtDynamic     = true;   // renderer rebuilds the BLAS every frame
			rtMesh->rtAlphaTested = true;   // rays alpha-test texture x particle fade
			// Canonical quad UVs: registered ONCE in the RT concat buffers; world.ps reconstructs
			// these exact values analytically, so they must never change.
			static const float qu[12] = { 0,1, 1,1, 1,0, 0,1, 1,0, 0,0 };
			for (int i = 0; i < cap; ++i) memcpy(rtMesh->uvArray + (size_t)i * 12, qu, sizeof(qu));
			for (int i = 0; i < cap * 6; ++i)
			{ rtMesh->normalArray[i * 3] = 0.f; rtMesh->normalArray[i * 3 + 1] = 0.f; rtMesh->normalArray[i * 3 + 2] = 1.f; }
			std::cout << "[VFX]\t\tparticle RT quads active (cap " << cap << ")" << std::endl;
		}
		// stretched sprites shadow as their full quad; round billboards as a disc
		rtMesh->rtShadowShape = (baseMode == 1) ? 0 : 1;

		float* v  = rtMesh->vertexArray;
		float* vc = rtMesh->rtColorArray;
		int q = 0;
		for (int i = 0; i < n; ++i)
		{
			const P& p = parts[i];
			float lt = 1.f - p.life / p.maxLife;
			float col[4] = { (float)startColor.r, (float)startColor.g, (float)startColor.b,
			                 (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f)) };
			EvalGradient(colorGradient, lt, col);
			if (col[3] < 0.02f) continue;   // fully faded -> out of the TLAS
			alphaSum += col[3]; ++alphaN;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
			if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
			glm::vec3 rv, uv2;
			if (baseMode == 1)   // velocity-stretched (same math as Draw)
			{
				glm::vec3 vel(p.vel[0], p.vel[1], p.vel[2]);
				float spd = glm::length(vel);
				glm::vec3 axis = spd > 1e-4f ? vel / spd : up;
				glm::vec3 side = glm::normalize(glm::cross(axis, camF) + glm::vec3(1e-5f));
				rv = side * (sz * 0.5f); uv2 = axis * (sz * 0.5f + spd * stretch);
			}
			else                 // camera-facing billboard (rotated)
			{
				float cr = cosf(p.rot), sr = sinf(p.rot);
				rv  = (right * cr + up * sr) * (sz * 0.5f);
				uv2 = (up * cr - right * sr) * (sz * 0.5f);
			}
			const glm::vec3 v0 = wp - rv - uv2, v1 = wp + rv - uv2, v2 = wp + rv + uv2, v3 = wp - rv + uv2;
			const glm::vec3 quad[6] = { v0, v1, v2, v0, v2, v3 };
			float* dst = v + (size_t)q * 18;
			for (int k = 0; k < 6; ++k) { dst[k * 3] = quad[k].x; dst[k * 3 + 1] = quad[k].y; dst[k * 3 + 2] = quad[k].z; }
			const float gB = 1.f + glow * EvalCurve(glowOverLife, lt, 1.f);
			float* cdst = vc + (size_t)q * 24;
			for (int k = 0; k < 6; ++k) { cdst[k * 4] = col[0] * gB; cdst[k * 4 + 1] = col[1] * gB; cdst[k * 4 + 2] = col[2] * gB; cdst[k * 4 + 3] = col[3]; }
			++q;
		}
		memset(v  + (size_t)q * 18, 0, ((size_t)cap - q) * 18 * sizeof(float));
		memset(vc + (size_t)q * 24, 0, ((size_t)cap - q) * 24 * sizeof(float));   // a=0 -> any-hit ignores
		if (q > 0)
		{
			rtMesh->version++;
			// Black albedo + white emissive: the per-vertex color pool carries the tint/fade.
			// Zero specular so rays never recurse off a particle; diff feeds the any-hit test.
			if (!rtMat) rtMat = new Material();
			Texture* baseTex = texCache ? texCache : ShapeTex(spriteShape);
			rtMat->diff  = baseTex;
			rtMat->em    = baseTex;
			rtMat->color = Color(0, 0, 0, alphaN ? (double)(alphaSum / alphaN) : 1.0);   // a = shadow gate
			rtMat->metallic = 0.f; rtMat->roughness = 1.f; rtMat->specular = 0.f;
			rtMat->emissive = Color(1, 1, 1, 1);
			rtMat->emissiveIntensity = 1.0f;   // the glow boost is per-vertex (color pool)
			float pos[3] = { 0, 0, 0 }, quat[4] = { 0, 0, 0, 1 }, scale[3] = { 1, 1, 1 };   // verts are world-space
			r->addRTInstance(rtMesh, rtMat, pos, quat, scale, inReflections, castShadows);
		}
	}

	// ---- TRAIL ribbons: a second quad mesh (real ribbon UVs, strip shadow footprint) ----
	if (wantTrail && trailHist.size() == parts.size() * (size_t)kTrailCap * 3)
	{
		const int segs = trailSegments < 2 ? 2 : (trailSegments > kTrailCap ? kTrailCap : trailSegments);
		const int quadsPer = segs - 1;
		const int tcap = cap * quadsPer;
		if (!rtTrailMesh || rtTrailCap != tcap)
		{
			FreeOneRTMesh(rtTrailMesh, rtTrailMat, rtTrailCap);
			rtTrailMesh = new Mesh();
			rtTrailCap  = tcap;
			strncpy(rtTrailMesh->name, "particles_trail_rt", sizeof(rtTrailMesh->name) - 1);
			rtTrailMesh->numVerts     = tcap * 6;
			rtTrailMesh->vertexArray  = new float[(size_t)tcap * 18]();
			rtTrailMesh->normalArray  = new float[(size_t)tcap * 18];
			rtTrailMesh->uvArray      = new float[(size_t)tcap * 12];
			rtTrailMesh->rtColorArray = new float[(size_t)tcap * 24]();
			rtTrailMesh->rtDynamic     = true;
			rtTrailMesh->rtAlphaTested = true;
			rtTrailMesh->rtShadowShape = 2;   // ribbon: strip across u
			for (int i = 0; i < tcap * 6; ++i)
			{ rtTrailMesh->normalArray[i * 3] = 0.f; rtTrailMesh->normalArray[i * 3 + 1] = 0.f; rtTrailMesh->normalArray[i * 3 + 2] = 1.f; }
			// Ribbon UVs are fixed per quad slot (u across, v head->tail): the concat UV
			// registration is one-shot, so they are written once and never updated.
			for (int qi = 0; qi < tcap; ++qi)
			{
				const int sgi = qi % quadsPer;                                  // segment within the ribbon
				const float vA = (float)(quadsPer - sgi)     / (float)quadsPer; // alongA (older end)
				const float vB = (float)(quadsPer - sgi - 1) / (float)quadsPer; // alongB (newer end)
				float* u = rtTrailMesh->uvArray + (size_t)qi * 12;
				u[0]=0; u[1]=vA;  u[2]=1; u[3]=vA;  u[4]=1; u[5]=vB;
				u[6]=0; u[7]=vA;  u[8]=1; u[9]=vB;  u[10]=0; u[11]=vB;
			}
		}
		float* tv  = rtTrailMesh->vertexArray;
		float* tvc = rtTrailMesh->rtColorArray;
		int tq = 0;
		for (int i = 0; i < n; ++i)
		{
			const P& p = parts[i];
			float lt = 1.f - p.life / p.maxLife;
			float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
			float col[4] = { (float)startColor.r, (float)startColor.g, (float)startColor.b,
			                 (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f)) };
			EvalGradient(colorGradient, lt, col);
			if (col[3] < 0.02f) continue;
			const float* h = trailHist.data() + (size_t)i * kTrailCap * 3;
			const float headW = (sz * trailWidth * 0.5f) < 0.001f ? 0.001f : (sz * trailWidth * 0.5f);
			for (int sgi = kTrailCap - segs; sgi < kTrailCap - 1 && tq < tcap; ++sgi)
			{
				glm::vec3 a(h[sgi * 3], h[sgi * 3 + 1], h[sgi * 3 + 2]);
				glm::vec3 b(h[(sgi + 1) * 3], h[(sgi + 1) * 3 + 1], h[(sgi + 1) * 3 + 2]);
				if (localSpace) { a = glm::vec3(l2w * glm::vec4(a, 1)); b = glm::vec3(l2w * glm::vec4(b, 1)); }
				glm::vec3 d = b - a;
				float* dst  = tv  + (size_t)tq * 18;
				float* cdst = tvc + (size_t)tq * 24;
				if (glm::dot(d, d) < 1e-10f) { memset(dst, 0, 18 * sizeof(float)); memset(cdst, 0, 24 * sizeof(float)); ++tq; continue; }
				glm::vec3 side = glm::normalize(glm::cross(glm::normalize(d), camF) + glm::vec3(1e-6f));
				const float alongA = (float)(kTrailCap - 1 - sgi) / (float)(segs - 1);
				const float alongB = (float)(kTrailCap - 2 - sgi) / (float)(segs - 1);
				const float wA = headW * (1.f - trailTaper * alongA);
				const float wB = headW * (1.f - trailTaper * alongB);
				const float aA = col[3] * (1.f - trailFade * alongA);
				const float aB = col[3] * (1.f - trailFade * alongB);
				glm::vec3 s0 = side * wA, s1 = side * wB;
				const glm::vec3 q0 = a - s0, q1 = a + s0, q2 = b + s1, q3 = b - s1;
				const glm::vec3 quad[6] = { q0, q1, q2, q0, q2, q3 };
				const float av[6] = { aA, aA, aB, aA, aB, aB };
				const float gB = 1.f + glow * EvalCurve(glowOverLife, lt, 1.f);
				for (int k = 0; k < 6; ++k)
				{
					dst[k * 3] = quad[k].x; dst[k * 3 + 1] = quad[k].y; dst[k * 3 + 2] = quad[k].z;
					cdst[k * 4] = col[0] * gB; cdst[k * 4 + 1] = col[1] * gB; cdst[k * 4 + 2] = col[2] * gB; cdst[k * 4 + 3] = av[k];
				}
				++tq;
			}
		}
		memset(tv  + (size_t)tq * 18, 0, ((size_t)tcap - tq) * 18 * sizeof(float));
		memset(tvc + (size_t)tq * 24, 0, ((size_t)tcap - tq) * 24 * sizeof(float));
		if (tq > 0)
		{
			rtTrailMesh->version++;
			if (!rtTrailMat) rtTrailMat = new Material();
			rtTrailMat->diff  = trailTexCache;   // null = plain ribbon (any-hit passes on fade only)
			rtTrailMat->em    = trailTexCache;
			rtTrailMat->color = Color(0, 0, 0, alphaN ? (double)(alphaSum / alphaN) : 1.0);
			rtTrailMat->metallic = 0.f; rtTrailMat->roughness = 1.f; rtTrailMat->specular = 0.f;
			rtTrailMat->emissive = Color(1, 1, 1, 1);
			rtTrailMat->emissiveIntensity = 1.0f;   // glow boost is per-vertex (color pool)
			float pos[3] = { 0, 0, 0 }, quat[4] = { 0, 0, 0, 1 }, scale[3] = { 1, 1, 1 };
			r->addRTInstance(rtTrailMesh, rtTrailMat, pos, quat, scale, inReflections, castShadows);
		}
	}
}

// Publishes this frame's particle light(s): lightCount 0 = one per particle, 1 = a single light
// at the alpha-weighted centroid, N = the N biggest particles. Color = particle color x (1+glow).
void ParticleEmitter::SubmitLights()
{
	if (lightIntensity <= 0.f || parts.empty()) return;
	glm::mat4 l2w(1.0f);
	if (localSpace && transform)
	{
		Vector3 cp = transform->globalPosition(); Quaternion Q = transform->globalRotation();
		l2w = glm::translate(glm::mat4(1.f), glm::vec3((float)cp.x, (float)cp.y, (float)cp.z))
		    * glm::mat4_cast(glm::quat((float)Q.w, (float)Q.x, (float)Q.y, (float)Q.z));
	}
	const float gI = 1.f + (glow > 0.f ? glow : 0.f);
	struct LP { float w; float m; glm::vec3 p; float c[3]; };
	static std::vector<LP> lps; lps.clear();
	for (const P& p : parts)
	{
		float lt = 1.f - p.life / p.maxLife;
		float col[4] = { (float)startColor.r, (float)startColor.g, (float)startColor.b,
		                 (float)startColor.a * Clamp01(EvalCurve(alphaOverLife, lt, 1.f)) };
		EvalGradient(colorGradient, lt, col);
		float m = EvalCurve(glowOverLife, lt, 1.f);
		if (lightBindAlpha) m *= Clamp01(col[3]);
		if (m < 0.005f || col[3] < 0.005f) continue;   // fully faded out
		float sz = p.size * EvalCurve(sizeOverLife, lt, 1.f);
		glm::vec3 wp(p.pos[0], p.pos[1], p.pos[2]);
		if (localSpace) wp = glm::vec3(l2w * glm::vec4(wp, 1));
		lps.push_back({ col[3] * sz, m, wp, { col[0], col[1], col[2] } });
	}
	if (lps.empty()) return;
	auto submit = [&](const glm::vec3& pos, const float c[3], float m)
	{
		NukeLight nl; nl.type = 1;   // point
		nl.pos[0] = pos.x; nl.pos[1] = pos.y; nl.pos[2] = pos.z;
		nl.color[0] = c[0] * gI; nl.color[1] = c[1] * gI; nl.color[2] = c[2] * gI;
		nl.intensity = lightIntensity * m; nl.range = lightRadius; nl.castShadows = 0;
		FrameLights::Submit(nl);
	};
	if (lightCount <= 0)
	{
		// One light per particle; the renderer clamps the world set at its 256-light budget.
		for (const LP& l : lps) submit(l.p, l.c, l.m);
	}
	else if (lightCount == 1)
	{
		glm::vec3 cpos(0); float cw = 0, cm = 0; float cc[3] = { 0, 0, 0 };
		for (const LP& l : lps) { cpos += l.p * l.w; cw += l.w; cm += l.m * l.w; cc[0] += l.c[0] * l.w; cc[1] += l.c[1] * l.w; cc[2] += l.c[2] * l.w; }
		if (cw <= 0.f) return;
		cpos /= cw; float c[3] = { cc[0] / cw, cc[1] / cw, cc[2] / cw };
		submit(cpos, c, cm / cw);
	}
	else
	{
		const int want = lightCount > 256 ? 256 : lightCount;
		const int take = (int)lps.size() < want ? (int)lps.size() : want;
		std::partial_sort(lps.begin(), lps.begin() + take, lps.end(), [](const LP& a, const LP& b) { return a.w * a.m > b.w * b.m; });
		for (int i = 0; i < take; ++i) submit(lps[i].p, lps[i].c, lps[i].m);
	}
}

}  // namespace nuke
