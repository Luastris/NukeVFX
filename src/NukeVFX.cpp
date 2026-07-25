// NukeVFX — the particle/VFX module (Phase 7.3). A SEPARATE module by design: games without
// effects never pay for it. Model + sim + rendering live in Particles.h/.cpp; an EFFECT is a
// PREFAB carrying ParticleEmitter atoms (standard inspector/undo/serialization/mods).
#include <NukeVFX/Particles.h>

#include <interface/NUKEEInteface.h>
#include <interface/AtomCreators.h>   // "+"-menu atom templates (editor builds the menu from this)
#include <interface/ComponentIcons.h> // viewport entity icons (editor draws from this registry)
#include <iostream>
#include <cstring>

using namespace nuke;

// nukegen (module mode): reflection registration for ParticleEmitter + ForceField —
// generated into NukeVFX.gen.inc by the CMake prebuild, #included IN-TU.
#include "NukeVFX.gen.inc"   // defines NukeReflectInit_NukeVFX()

class NukeVFXModule : public NUKEModule
{
public:
	NukeVFXModule()
	{
		strcpy(title, "NukeVFX");
		strcpy(author, "Luastris");
		strcpy(version, "1.0");
		strcpy(description, "Particle VFX: emitters (billboard/stretched/trail/mesh), force fields, wind/physics interaction");
	}

	void OnLoad() override
	{
		NukeReflectInit_NukeVFX();   // ParticleEmitter/ForceField become Add Component-able / scriptable

		// "+"-menu atom templates: the editor instantiates the named components through
		// reflection — no editor<->module linkage.
		RegisterAtomCreator({ "Effects", "Particle Emitter", "" /* ICON_LC_SPARKLES */, { "ParticleEmitter" } });
		RegisterAtomCreator({ "Effects", "Force Field",      "" /* ICON_LC_MAGNET */,   { "ForceField" } });
		// Viewport entity icons for OUR component types (clickable, like lights/cameras).
		RegisterComponentIcon({ "ParticleEmitter", "" /* ICON_LC_SPARKLES */, { 1.0f, 0.75f, 0.35f, 0.92f } });
		RegisterComponentIcon({ "ForceField",      "" /* ICON_LC_MAGNET */,   { 0.55f, 0.82f, 1.0f, 0.92f } });
		std::cout << "[NukeVFX]\tloaded" << std::endl;
	}

	void Run(AppInstance*) override {}
	void Shutdown() override { stopped = true; }
	bool HasSettings() override { return false; }
	void Settings() override {}
};

// Exported under the unmangled symbol "plugin" — the loader imports it via boost::dll.
extern "C" __declspec(dllexport) NukeVFXModule plugin;
NukeVFXModule plugin;
