#pragma once
#include <NukeVFX/ScatterVolume.h>
#include <API/Model/Color.h>

namespace nuke {

// Fog: a ScatterVolume whose medium also has extinction - it dims and tints what lies behind
// it (density / albedo), may glow (emission), and scatters the lights like any scatter volume.
class NUKEVFX_API FogVolume : public ScatterVolume
{
	NUKE_CLASS(FogVolume, ScatterVolume, "Effects")
public:
	[[nuke::prop(label="Density", min=0, tip="Extinction inside the volume, 1/m (0.1 = dense mist, 1 = thick smoke).")]] float density = 0.1f;
	[[nuke::prop(label="Albedo", tip="Colour of the light this fog scatters.")]] Color albedo = Color(0.9, 0.9, 0.9, 1.0);
	[[nuke::prop(label="Emission", tip="Light the fog emits by itself (glowing gas); scaled by Emission Intensity.")]] Color emission = Color(1.0, 0.6, 0.2, 1.0);
	[[nuke::prop(label="Emission Intensity", min=0)]] float emissionIntensity = 0.0f;

	FogVolume();
	void FillMedium(NukeFogVolumeDesc& d) const override;
	bool HasMedium() const override { return density > 0.0f || scattering > 0.0f || emissionIntensity > 0.0f; }
};

}  // namespace nuke
