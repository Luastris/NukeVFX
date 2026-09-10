#include <NukeVFX/FogVolume.h>
#include <render/irender.h>
#include <algorithm>

namespace nuke {

FogVolume::FogVolume() : ScatterVolume("FogVolume") {}

void FogVolume::FillMedium(NukeFogVolumeDesc& d) const
{
	ScatterVolume::FillMedium(d);
	d.density = std::max(density, 0.0f);
	d.albedo[0] = (float)albedo.r; d.albedo[1] = (float)albedo.g; d.albedo[2] = (float)albedo.b;
	d.emission[0] = (float)emission.r * emissionIntensity;
	d.emission[1] = (float)emission.g * emissionIntensity;
	d.emission[2] = (float)emission.b * emissionIntensity;
}

}  // namespace nuke
