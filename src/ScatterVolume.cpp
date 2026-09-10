#include <NukeVFX/ScatterVolume.h>
#include <render/irender.h>
#include <algorithm>

namespace nuke {

ScatterVolume::ScatterVolume() : MediumVolume("ScatterVolume") {}
ScatterVolume::ScatterVolume(const char* typeName) : MediumVolume(typeName) {}

void ScatterVolume::FillMedium(NukeFogVolumeDesc& d) const
{
	MediumVolume::FillMedium(d);
	d.density = 0.0f;                                   // a scatter volume dims nothing
	d.albedo[0] = d.albedo[1] = d.albedo[2] = 1.0f;
	d.emission[0] = d.emission[1] = d.emission[2] = 0.0f;
	d.noise = noise; d.noiseScale = std::max(noiseScale, 0.1f); d.windAdvect = windAdvect;
	d.shaftDensity = std::max(scattering, 0.0f);
	d.heightFalloff = std::max(heightFalloff, 0.0f);
	d.fluidMode = fluidMode; d.clumpSize = std::max(clumpSize, 0.0f);
	d.fluid = fluid ? 1 : 0;
	d.fluidRes = std::max(8, std::min(192, fluidResolution));
	d.fluidTurbulence = std::max(fluidTurbulence, 0.0f); d.fluidRefill = std::max(fluidRefill, 0.0f); d.fluidDissipation = std::max(fluidDissipation, 0.0f);
}

}  // namespace nuke
