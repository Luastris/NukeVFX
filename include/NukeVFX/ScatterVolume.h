#pragma once
#include <API/Model/MediumVolume.h>

#ifdef NUKEVFX_EXPORTS
  #define NUKEVFX_API __declspec(dllexport)
#else
  #define NUKEVFX_API __declspec(dllimport)
#endif

namespace nuke {

// A local light-scattering medium: clear air inside the shape that scatters the lights toward
// the eye - the sun's rays through the shadows, a spot's cone, a lamp's glow - without dimming
// anything behind it. Eroded by 3D noise that drifts with the global wind; optionally a fluid
// (wind, bodies, Force Fields). FogVolume derives from it and adds the extinction (real fog).
// Lives in the World's volumetric grid (World Settings > Volumetrics); additive with the global
// medium and every other volume.
class NUKEVFX_API ScatterVolume : public MediumVolume
{
	NUKE_CLASS(ScatterVolume, MediumVolume, "Effects")
public:
	[[nuke::prop(label="Light Scattering", min=0, tip="How much the air inside scatters the lights toward the eye, per metre (0.05 = clear beams, 0.5 = strong cones around a lamp). Nothing behind it dims.")]] float scattering = 0.05f;
	[[nuke::prop(label="Height Falloff", min=0, tip="Like the global fog: the medium thins exponentially above the shape's bottom, 1/m (0 = fills the shape evenly). Make the shape taller than the fog, and force fields above it can lift it.")]] float heightFalloff = 0.0f;
	[[nuke::prop(label="Noise", min=0, max=1, tip="Erosion by 3D noise: 0 = solid, 1 = fully broken into wisps.")]] float noise = 0.0f;
	[[nuke::prop(label="Noise Scale", min=0.1, tip="World units per noise feature.")]] float noiseScale = 4.0f;
	[[nuke::prop(label="Wind Advection", min=0, tip="The noise drifts with the global wind: 1 = at wind speed, 0 = still.")]] float windAdvect = 1.0f;
	// Fluid: the medium becomes a simulated 3D field inside the shape - it rolls with the wind,
	// is pushed, pulled and swirled by Force Fields, parts around whatever walks through it
	// (CharacterControllers, Rigidbodies), stirs by itself, and refills toward the shape.
	[[nuke::prop(label="Fluid", tip="Simulate the medium as a 3D fluid inside the shape: it rolls with the wind, obeys Force Fields (attract / repel / vortex / turbulence), parts around characters and bodies moving through it, and refills toward the shape.")]] bool fluid = false;
	[[nuke::prop(label="Fluid Mode", enum="Grid,Clumps", tip="Grid: the density flows on the grid (wind, wakes, holes); a vortex shapes it into a funnel. Clumps: the medium is parcels carried by the flow - they die in a vortex's drain and are born at the edges.")]] int fluidMode = 0;
	[[nuke::prop(label="Clump Size", min=0, tip="Clumps: a parcel's radius in metres (0 = from the shape's size). Smaller = finer, more parcels.")]] float clumpSize = 0.0f;
	[[nuke::prop(label="Fluid Resolution", min=8, max=192, tip="Simulation cells along the shape's longest axis (48 = smooth swirls at a few metres, 96 = fine detail; cost grows with the cube).")]] int fluidResolution = 48;
	[[nuke::prop(label="Turbulence", min=0, tip="Self-stirring of the fluid, m/s (0 = only wind and bodies move it).")]] float fluidTurbulence = 0.3f;
	[[nuke::prop(label="Refill", min=0, tip="How fast pushed-away or blown-away medium comes back, per second (0.5 = a couple of seconds). Clumps: how fast a newborn clump gains its weight.")]] float fluidRefill = 0.5f;
	[[nuke::prop(label="Dissipation", min=0, tip="Density lost per second while it drifts (0 = none). Clumps: how fast a clump dissolves in a drain or beyond the shape.")]] float fluidDissipation = 0.1f;

	ScatterVolume();
	explicit ScatterVolume(const char* typeName);
	void FillMedium(NukeFogVolumeDesc& d) const override;
	bool HasMedium() const override { return scattering > 0.0f; }
};

}  // namespace nuke
