#pragma once
// NukeVFX — particle system: CPU sim on nuke::Jobs (wind, force fields, raycast collision,
// sub-emitters), rendering via billboards/trails (drawSpriteRun) and mesh instancing.
// An effect is a PREFAB whose atoms carry ParticleEmitter components.
#ifndef NUKEVFX_PARTICLES_H
#define NUKEVFX_PARTICLES_H

#include <API/Model/Include.h>
#include <API/Model/Vector.h>
#include <API/Model/Color.h>
#include <reflect/Reflect.h>
#include <vector>
#include <string>

#ifdef _WIN32
  #ifdef NUKEVFX_EXPORTS
  #define NUKEVFX_API __declspec(dllexport)
  #else
  #define NUKEVFX_API __declspec(dllimport)
  #endif
#else
  #define NUKEVFX_API __attribute__((visibility("default")))
#endif

namespace nuke {

// The emitter. Curves are flattened key arrays over normalized life 0..1; the color gradient
// is flattened (t,r,g,b) stops.
class NUKEVFX_API ParticleEmitter : public Component
{
	// OnRender fires per camera pass; the editor-preview sim step must happen once a frame.
	unsigned long long previewStepFrame = ~0ull;

	NUKE_CLASS(ParticleEmitter, Component, "Effects")
public:
	// ---- emission --------------------------------------------------------------------------
	[[nuke::prop(label="Playing")]]                     bool  playing = true;
	[[nuke::prop(label="Duration", min=0, tip="Seconds of emission; 0 = loop forever.")]] float duration = 0.0f;
	[[nuke::prop(label="Max Particles", min=1)]]        int   maxParticles = 1000;
	[[nuke::prop(label="Rate", min=0, tip="Particles per second.")]]             float rate = 50.0f;
	[[nuke::prop(label="Burst Count", min=0, tip="Particles per burst (0 = no bursts).")]] int burstCount = 0;
	[[nuke::prop(label="Burst Interval", min=0.01)]]    float burstInterval = 1.0f;
	// ---- shape -----------------------------------------------------------------------------
	[[nuke::prop(label="Shape", enum="Point,Sphere,Box,Cone,MeshSurface,CameraBox", tip="CameraBox: a box that rides the MAIN camera (Shape Extents around Camera Offset), particles spawn falling - rain, snow, wind-blown dust around the viewer.")]] int shape = 1;
	[[nuke::prop(label="Shape Radius", min=0)]]         float shapeRadius = 0.5f;      // sphere/cone base
	[[nuke::prop(label="Shape Extents")]]               Vector3 shapeExtents = Vector3(0.5, 0.5, 0.5);   // box half
	[[nuke::prop(label="Camera Offset", tip="CameraBox: the box centre relative to the camera - x right, y up, z ahead (flattened).")]] Vector3 cameraOffset = Vector3(0, 6, 4);
	[[nuke::prop(label="Cone Angle", min=0, max=89)]]   float coneAngle = 25.0f;       // deg from +Y
	[[nuke::prop(label="From Shell", tip="Emit from the shape surface instead of the volume.")]] bool fromShell = false;
	[[nuke::prop(asset="mesh", label="Emit Mesh", tip="MeshSurface shape: emit from this mesh's triangles. Empty = this atom's own rendered mesh, else the parent atom's (prefab flames riding a burning object).")]] std::string emitMeshGuid;
	[[nuke::prop(label="Emit Mask", tip="MeshSurface shape: emit only where this painted surface condition is hot on the source atom (e.g. \"burn\" - flames track the fire front and grow with it). Cold samples are skipped, so emission scales with the hot area. Empty = the whole surface.")]] std::string emitMask;
	// ---- initial ranges --------------------------------------------------------------------
	[[nuke::prop(label="Life Min", min=0.01)]]          float lifeMin = 1.0f;
	[[nuke::prop(label="Life Max", min=0.01)]]          float lifeMax = 2.0f;
	[[nuke::prop(label="Speed Min")]]                   float speedMin = 1.0f;
	[[nuke::prop(label="Speed Max")]]                   float speedMax = 3.0f;
	[[nuke::prop(label="Size Min", min=0.001)]]         float sizeMin = 0.1f;
	[[nuke::prop(label="Size Max", min=0.001)]]         float sizeMax = 0.25f;
	[[nuke::prop(label="Rotation Speed", tip="Random spin, +- deg/sec.")]]        float rotSpeed = 0.0f;
	[[nuke::prop(label="Inherit Velocity", min=0, max=1, tip="Fraction of the atom's velocity added at spawn.")]] float inheritVelocity = 0.0f;
	[[nuke::prop(label="Local Space", tip="Simulate in the atom's space (particles follow it).")]] bool localSpace = false;
	// ---- forces ----------------------------------------------------------------------------
	[[nuke::prop(label="Gravity", tip="Multiplier of world gravity.")]] float gravity = 0.0f;
	[[nuke::prop(label="Drag", min=0)]]                 float drag = 0.0f;
	[[nuke::prop(label="Wind Influence", min=0, max=1)]] float windInfluence = 0.0f;   // nuke::Wind::Sample
	// ---- collision -------------------------------------------------------------------------
	[[nuke::prop(label="Collision", enum="Off,World,Surface", tip="World = everything (physics). Surface = only the referenced atom's meshes.")]] int collision = 0;
	[[nuke::prop(label="Surface", tip="Surface mode: the one atom (subtree) particles collide with.")]] Atom* collisionSurface = nullptr;
	[[nuke::prop(label="Bounce", min=0, max=1)]]        float bounce = 0.3f;
	[[nuke::prop(label="Collision Dampen", min=0, max=1)]] float collideDampen = 0.2f;
	[[nuke::prop(label="Die On Collision")]]            bool  dieOnCollision = false;
	[[nuke::prop(label="Water Contact", tip="Particles hitting a water surface splash rings where they land (needs NukeWater loaded).")]] bool waterContact = false;
	[[nuke::prop(label="Die On Water", tip="Water Contact: remove the particle at the surface (off = a damped bounce).")]] bool dieOnWater = true;
	[[nuke::prop(label="Collision Events", tip="Emit 'vfx.collision' per hit: point, normal, hit atom, uv (json payload).")]] bool collisionEvents = false;
	[[nuke::prop(label="Collision Budget", min=0, tip="Max collision rays per frame; particles take turns and the ray reach stretches over skipped frames, so nothing tunnels. 0 = every particle every frame.")]] int collisionBudget = 512;
	// LiveMaterial integration: every collision fires the HIT SURFACE's own reaction
	// (particles/decals/sound from ITS material); the particle's speed is the impulse.
	[[nuke::prop(label="Surface Hits", tip="Collisions fire the hit surface's LiveMaterial reaction (its own decal/prefab/sound)")]] bool surfaceHits = false;
	[[nuke::prop(label="Hit Type", tip="Typed hit id sent to the surface (empty matches its any-hit entry)")]] std::string surfaceHitType;
	// ---- over-lifetime ---------------------------------------------------------------------
	[[nuke::prop(label="Size Over Life", widget="curve")]]  std::vector<float> sizeOverLife;    // keys (t,v,inTan,outTan)
	[[nuke::prop(label="Alpha Over Life", widget="curve", min=0, max=1)]] std::vector<float> alphaOverLife;   // keys (t,v,inTan,outTan); alpha is 0..1

	[[nuke::prop(label="Color Gradient", widget="gradient")]] std::vector<float> colorGradient; // (t,r,g,b) stops
	[[nuke::prop(label="Start Color")]]                 Color startColor = Color(1, 1, 1, 1);
	// ---- render ----------------------------------------------------------------------------
	// Value 2 is legacy: it draws billboard + trail (the trail is an option, not a mode).
	[[nuke::prop(label="Render", enum="Billboard,Stretched,Billboard + Trail,Mesh")]] int renderMode = 0;
	[[nuke::prop(asset="texture", label="Texture")]]    std::string textureGuid;   // billboard/stretched
	[[nuke::prop(label="Particle Shape", enum="Quad,Circle,Ring,Spark,Star,Smoke", tip="Built-in shape when no Texture is set (procedural, no asset needed).")]] int spriteShape = 1;
	[[nuke::prop(asset="mesh", label="Particle Mesh")]] std::string meshGuid;      // Mesh mode (7.1 instancing)
	[[nuke::prop(asset="material", label="Mesh Material")]] std::string materialGuid;
	[[nuke::prop(label="Blend", enum="Alpha,Additive")]] int blend = 0;
	[[nuke::prop(label="Stretch", min=0, tip="Stretched mode: length per unit of speed.")]] float stretch = 0.1f;
	[[nuke::prop(label="Sort Particles", tip="Back-to-front (alpha blend correctness; costs CPU).")]] bool sortParticles = false;
	// ---- trail (option on top of ANY render mode) ------------------------------------------
	[[nuke::prop(label="Trail", tip="Draw a ribbon behind each particle, on top of the base render.")]] bool trailEnabled = false;
	[[nuke::prop(label="Trail Segments", min=2, max=32)]] int trailSegments = 8;
	[[nuke::prop(label="Trail Width", min=0, tip="Ribbon width as a fraction of the particle size.")]] float trailWidth = 0.5f;
	[[nuke::prop(label="Trail Taper", min=0, max=1, tip="1 = ribbon narrows to a point at the tail, 0 = constant width.")]] float trailTaper = 1.0f;
	[[nuke::prop(label="Trail Fade", min=0, max=1, tip="How much the ribbon fades out toward the tail.")]] float trailFade = 1.0f;
	[[nuke::prop(asset="texture", label="Trail Texture", tip="Stretched ALONG the whole ribbon (u across, v head->tail). Empty = plain ribbon.")]] std::string trailTextureGuid;
	[[nuke::prop(label="Soft Fade", min=0, tip="Soft particles: fade within this distance of scene geometry (needs a depth prepass - TAA/SSR/decals on the camera). 0 = off.")]] float softFade = 0.0f;
	// ---- volumetric fog interplay (World Settings > Volumetric Fog) ---------------------------
	[[nuke::prop(label="Volumetric Lighting", min=0, max=1, tip="Light the sprites by the froxel fog grid (sun through its shadows, point/spot, probe ambient): 1 = the grid's light replaces the authored brightness - smoke sits in the god rays. Needs Volumetric Fog on.")]] float volumeLight = 0.0f;
	[[nuke::prop(label="Six-Way Lighting", tip="Hero smoke: two lightmaps hold the puff lit from six directions (A.rgb = +right/-right/+up, B.rgb = -up/+front/-front, A.a = opacity); every scene light shades the puff by its direction.")]] bool sixWay = false;
	[[nuke::prop(asset="texture", label="Six-Way Map A")]] std::string sixWayGuidA;
	[[nuke::prop(asset="texture", label="Six-Way Map B")]] std::string sixWayGuidB;
	// ---- lighting / ray tracing ------------------------------------------------------------
	[[nuke::prop(label="Glow", min=0, tip="HDR emissive boost: particle color is multiplied by (1 + glow), so bloom picks it up and reflected particles glow too. 0 = off.")]] float glow = 0.0f;
	[[nuke::prop(label="Glow Over Life", widget="curve", min=0, tip="Multiplier of Glow (and of the particle light) over the particle's life — emission breathes in and out instead of popping.")]] std::vector<float> glowOverLife;   // keys (t,v,inTan,outTan)
	[[nuke::prop(label="In Reflections", tip="Ray-traced reflections show the particles (alpha-tested, per-particle color and fade).")]] bool inReflections = true;
	[[nuke::prop(label="Cast Shadows", tip="Particles occlude ray-traced light (sprites shadow as discs, stretched as quads, trails as ribbons). Raster shadow maps are unaffected.")]] bool castShadows = false;
	[[nuke::prop(label="Light", min=0, tip="Particles LIGHT THE SCENE: intensity of the emitted point light(s), 0 = off. Color follows the current particle color x (1 + glow).")]] float lightIntensity = 0.0f;
	[[nuke::prop(label="Light Radius", min=0.1)]] float lightRadius = 6.0f;
	[[nuke::prop(label="Light Particles", min=0, max=256, tip="0 = EVERY particle carries its own light (UE-style); 1 = one aggregated light at the cloud's center; N = the N biggest particles. The engine budget is 256 lights per world (extras are dropped).")]] int lightCount = 0;
	[[nuke::prop(label="Bind Light To Alpha", tip="The particle's alpha directly scales its light — fading particles dim smoothly instead of switching off. Glow Over Life scales the light too, always.")]] bool lightBindAlpha = true;
	// ---- sub-emitter -----------------------------------------------------------------------
	[[nuke::prop(label="Sub Emitter", tip="Another atom's ParticleEmitter: burst there when a particle dies/collides.")]] Atom* subEmitter = nullptr;
	[[nuke::prop(label="Sub Emitter Count", min=0)]]    int subEmitterCount = 5;
	[[nuke::prop(label="Sub On Collision", tip="Trigger the sub emitter on collision instead of death.")]] bool subOnCollision = false;

	// ---- reflected control (scripts) -------------------------------------------------------
	[[nuke::func]] void Play();
	[[nuke::func]] void Stop();                 // stop emitting (alive particles finish)
	[[nuke::func]] void Clear();                // kill every particle now
	[[nuke::func]] void Burst(double count);    // immediate burst at the emitter
	[[nuke::func]] void BurstAt(const Vector3& worldPos, double count);   // burst at a point (sub-emitter channel)
	[[nuke::func]] int  AliveCount();

	// ---- runtime ---------------------------------------------------------------------------
	struct P { float pos[3]; float vel[3]; float life; float maxLife; float size; float rot; float rotVel; float seed; float wet = 0.0f; };   // wet: below the water surface last step (a splash is an ENTRY, not every frame)
	std::vector<P> parts;               // alive particles (swap-erase on death)
	float emitAccum = 0.f, burstAccum = 0.f, ageSec = 0.f;
	double lastPos[3] = { 0, 0, 0 }; bool hasLastPos = false;   // atom velocity (inherit)
	uint64_t instBuf = 0; iRender* instOwner = nullptr;         // Mesh mode instance buffer
	// Trail history: kTrailCap*3 floats per particle, index-parallel with `parts`;
	// [kTrailCap-1] = the newest point.
	static const int kTrailCap = 32;
	std::vector<float> trailHist;
	// Asset caches track the guid they were resolved FROM — never latch, a changed/cleared
	// prop must re-resolve on the next draw.
	Texture* texCache = nullptr; Mesh* meshCache = nullptr; Material* matCache = nullptr;
	Material* meshTexMat = nullptr;   // Mesh mode without a Mesh Material: the Texture prop as albedo (owned)
	Material* meshGbufMat = nullptr;  // the G-buffer prepass copy: fully rough (reflections are traced on water/mirrors, not on particles)
	Material* MeshMaterial();
	int UploadMeshInstances(iRender* r);   // Mesh mode: pack + upload this frame's instance records (count)
	Texture* trailTexCache = nullptr;
	Texture* sixACache = nullptr; Texture* sixBCache = nullptr;
	std::string texGuidRes, meshGuidRes, matGuidRes, trailTexGuidRes, sixAGuidRes, sixBGuidRes;
	// RT quad meshes (reflections/shadow rays): sprite quads + trail ribbons, world-space verts
	// and per-vertex colors rewritten every frame; dead space = degenerate triangles. Owned material
	// color.a = average alive alpha (shadow gate).
	Mesh*     rtMesh = nullptr;      Material* rtMat = nullptr;      int rtCap = 0;
	Mesh*     rtTrailMesh = nullptr; Material* rtTrailMat = nullptr; int rtTrailCap = 0;
	void ResolveAssets();               // guid -> cache hot-apply (shared by Draw and the RT build)
	void BuildRTQuads(iRender* r);      // RenderPhase::RTScene: refresh quads/trails/mesh instances
	void SubmitLights();                // end of Advance: publish this frame's particle light(s)
	void FreeRTMesh();                  // invalidateMesh + delete (component teardown / capacity change)
	int rayCursor = 0;                   // editor-preview scene raycasts: round-robin budget start
	double pendingSubBursts = 0;         // deaths this frame -> sub-emitter bursts (applied on game thread)
	std::vector<float> subBurstPts;      // xyz per pending burst

	ParticleEmitter();
	void Init(Atom* parent) override;
	void Destroy() override;
	void Update() override;              // sim step (playing)
	void FixedUpdate() override;
	void Pause() override;
	void Reset() override;
	void OnRender(iRender* r, RenderPhase phase) override;   // edit-mode sim + draw (Transparent)

	void Advance(float dt);              // one sim step (spawn + integrate + collide + expire)
	void Draw(iRender* r);
};

}  // namespace nuke

#endif // !NUKEVFX_PARTICLES_H
