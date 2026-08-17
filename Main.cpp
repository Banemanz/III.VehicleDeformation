/*
    VehDeformIII
    GTA III 1.0 EN vehicle mesh deformation.

    Design:
      - GTA III collision/VehicleDamage remains authoritative.
      - Every deformable atomic gets private geometry; model-shared geometry is untouched.
      - One exact vehicle-local collision contact drives a spherical plastic dent over
        immutable rest vertices, transformed through each component's current frame.
      - Wheel meshes are never deformed. GTA III remains authoritative for suspension/steering;
        optional wheel translation is applied only around the final CEntity::Render call.
      - Repair restores the private geometry before CAutomobile::Fix.
      - The INI is created only when absent and is never overwritten.
*/

#include "plugin.h"

#include "common.h"
#include "CAutomobile.h"
#include "CPhysical.h"
#include "CColModel.h"
#include "CTimer.h"
#include "Patch.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace plugin;

#pragma comment(lib, "plugin_iii.lib")

namespace VehDeformIII {

// GTA III 1.0 EN, verified against the supplied GTA III IDB/function list.
constexpr uintptr_t kApplyCollisionTarget    = 0x4973A0;
constexpr uintptr_t kApplyCollisionAltTarget = 0x4992A0;
constexpr uintptr_t kVehicleDamageTarget     = 0x52F390;
constexpr uintptr_t kFixTarget               = 0x53C240;
constexpr uintptr_t kRwFrameGetLTMTarget     = 0x5A1CE0;
constexpr uintptr_t kRwFrameUpdateObjects    = 0x5A1C60;
constexpr uintptr_t kEntityRenderTarget      = 0x474BD0;

constexpr uintptr_t kVehicleDamageCall       = 0x531FE3;
constexpr uintptr_t kFixCall                 = 0x422787;
// Last CAutomobile::Render step before the clump is actually submitted. GTA III
// has finished suspension, steering and swinging-door frame updates at this point.
constexpr uintptr_t kAutomobileEntityRenderCall = 0x53B248;

constexpr float kEpsilon = 1.0e-5f;

struct Vec3 {
    float x{};
    float y{};
    float z{};
};

static Vec3 V(float x, float y, float z) { return { x, y, z }; }
static Vec3 Add(Vec3 a, Vec3 b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 Sub(Vec3 a, Vec3 b) { return V(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 Mul(Vec3 v, float s) { return V(v.x * s, v.y * s, v.z * s); }
static float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static float LengthSq(Vec3 v) { return Dot(v, v); }
static float Length(Vec3 v) { return std::sqrt(LengthSq(v)); }
static float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static Vec3 Normalize(Vec3 v, Vec3 fallback = {}) {
    const float len = Length(v);
    return len > kEpsilon ? Mul(v, 1.0f / len) : fallback;
}

static Vec3 FromRw(const RwV3d& v) { return V(v.x, v.y, v.z); }
static Vec3 FromC(const CVector& v) { return V(v.x, v.y, v.z); }

struct Config {
    bool enabled = true;
    bool onlyPlayerVehicle = false;

    // GTA III and Vice City share the same stock damage lineage here: both gate
    // VehicleDamage at 25, and both use the same >50 player-impact shake expression
    // (100 + 0.4 * impulse * 2000 / mass, capped at 250).  Keep the visual dent in
    // GTA III's raw damage-impulse domain; do not import the VC ASI's mass scaling.
    float minimumImpulse = 25.0f;
    float fullDeformationImpulse = 400.0f;
    float impulseExponent = 0.70f;
    float playerDeformationScale = 1.0f;
    float nonPlayerDeformationScale = 0.85f;

    // Direct-field baseline: localized spherical dent around the original/rest mesh.
    // Radius growth is the useful VC idea; axial capsules, rim pulling, ripple and
    // crinkle are intentionally absent from the baseline.
    float radius = 1.20f;
    float maximumRadius = 1.80f;
    float maximumDeformationPerImpact = 0.32f;
    float maximumTotalDeformation = 0.72f;
    float centerResistance = 0.40f;
    float inwardNormalBlend = 0.0f;

    // Kept isolated until the body path is stable.  GTA III rewrites wheel matrices
    // inside CAutomobile::Render, so this is a separate visual layer.
    bool wheelSync = false;
    float wheelSyncStrength = 0.45f;
    float wheelSyncRadiusScale = 1.10f;
    float wheelSyncMaxOffset = 0.16f;
    float wheelSyncVerticalScale = 0.15f;

    int maximumVerticesPerAtomic = 10000;
    int collisionSampleMaxAgeFrames = 2;
};

static Config gConfig;
static std::string gIniPath;

static const char kDefaultIni[] =
    "; VehDeformIII - GTA III 1.0 EN\r\n"
    "; Generated only when missing. Existing INIs are never overwritten.\r\n"
    "\r\n"
    "[VehDeformIII]\r\n"
    "Enabled=1\r\n"
    "OnlyPlayerVehicle=0\r\n"
    "\r\n"
    "; GTA III native damage-impulse scale. Stock VehicleDamage gates at 25.\r\n"
    "; III and VC share the same >50 shake formula; visual deformation stays in\r\n"
    "; III's raw impulse domain instead of importing VC ASI mass scaling.\r\n"
    "MinimumImpulse=25.0\r\n"
    "FullDeformationImpulse=400.0\r\n"
    "ImpulseExponent=0.70\r\n"
    "PlayerDeformationScale=1.0\r\n"
    "NonPlayerDeformationScale=0.85\r\n"
    "\r\n"
    "; Direct spherical dent over immutable rest vertices.\r\n"
    "Radius=1.20\r\n"
    "MaximumRadius=1.80\r\n"
    "MaximumDeformationPerImpact=0.32\r\n"
    "MaximumTotalDeformation=0.72\r\n"
    "CenterResistance=0.40\r\n"
    "InwardNormalBlend=0.0\r\n"
    "\r\n"
    "; Wheel sync is intentionally OFF in this body-deformation baseline. GTA III\r\n"
    "; remains authoritative for suspension/steering; enable only after body testing.\r\n"
    "WheelSync=0\r\n"
    "WheelSyncStrength=0.45\r\n"
    "WheelSyncRadiusScale=1.10\r\n"
    "WheelSyncMaxOffset=0.16\r\n"
    "WheelSyncVerticalScale=0.15\r\n"
    "\r\n"
    "MaximumVerticesPerAtomic=10000\r\n"
    "CollisionSampleMaxAgeFrames=2\r\n";

static std::string GetModulePath() {
    HMODULE module = nullptr;
    char path[MAX_PATH]{};
    GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        reinterpret_cast<LPCSTR>(&GetModulePath), &module);
    return module && GetModuleFileNameA(module, path, MAX_PATH) ? path : "VehDeformIII.asi";
}

static std::string ReplaceExtension(std::string path, const char* extension) {
    const size_t slash = path.find_last_of("\\/");
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return path + extension;
    path.erase(dot);
    return path + extension;
}

static void CreateIniIfMissing() {
    if (GetFileAttributesA(gIniPath.c_str()) != INVALID_FILE_ATTRIBUTES)
        return;

    HANDLE file = CreateFileA(gIniPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr,
        CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE)
        return;

    DWORD written{};
    WriteFile(file, kDefaultIni, static_cast<DWORD>(sizeof(kDefaultIni) - 1), &written, nullptr);
    CloseHandle(file);
}

static bool ReadBool(const char* key, bool fallback) {
    return GetPrivateProfileIntA("VehDeformIII", key, fallback ? 1 : 0, gIniPath.c_str()) != 0;
}

static int ReadInt(const char* key, int fallback) {
    return GetPrivateProfileIntA("VehDeformIII", key, fallback, gIniPath.c_str());
}

static float ReadFloat(const char* key, float fallback) {
    char fallbackText[64]{};
    char valueText[128]{};
    std::snprintf(fallbackText, sizeof(fallbackText), "%.9g", fallback);
    GetPrivateProfileStringA("VehDeformIII", key, fallbackText, valueText,
        static_cast<DWORD>(sizeof(valueText)), gIniPath.c_str());
    char* end = nullptr;
    const float value = std::strtof(valueText, &end);
    return end != valueText ? value : fallback;
}

static void LoadConfig() {
    gIniPath = ReplaceExtension(GetModulePath(), ".ini");
    CreateIniIfMissing();

    gConfig.enabled = ReadBool("Enabled", gConfig.enabled);
    gConfig.onlyPlayerVehicle = ReadBool("OnlyPlayerVehicle", gConfig.onlyPlayerVehicle);

    gConfig.minimumImpulse = ReadFloat("MinimumImpulse", gConfig.minimumImpulse);
    gConfig.fullDeformationImpulse = ReadFloat("FullDeformationImpulse", gConfig.fullDeformationImpulse);
    gConfig.impulseExponent = ReadFloat("ImpulseExponent", gConfig.impulseExponent);
    gConfig.playerDeformationScale = ReadFloat("PlayerDeformationScale", gConfig.playerDeformationScale);
    gConfig.nonPlayerDeformationScale = ReadFloat("NonPlayerDeformationScale", gConfig.nonPlayerDeformationScale);

    gConfig.radius = ReadFloat("Radius", gConfig.radius);
    gConfig.maximumRadius = ReadFloat("MaximumRadius", gConfig.maximumRadius);
    gConfig.maximumDeformationPerImpact = ReadFloat("MaximumDeformationPerImpact", gConfig.maximumDeformationPerImpact);
    gConfig.maximumTotalDeformation = ReadFloat("MaximumTotalDeformation", gConfig.maximumTotalDeformation);
    gConfig.centerResistance = ReadFloat("CenterResistance", gConfig.centerResistance);
    gConfig.inwardNormalBlend = ReadFloat("InwardNormalBlend", gConfig.inwardNormalBlend);

    gConfig.wheelSync = ReadBool("WheelSync", gConfig.wheelSync);
    gConfig.wheelSyncStrength = ReadFloat("WheelSyncStrength", gConfig.wheelSyncStrength);
    gConfig.wheelSyncRadiusScale = ReadFloat("WheelSyncRadiusScale", gConfig.wheelSyncRadiusScale);
    gConfig.wheelSyncMaxOffset = ReadFloat("WheelSyncMaxOffset", gConfig.wheelSyncMaxOffset);
    gConfig.wheelSyncVerticalScale = ReadFloat("WheelSyncVerticalScale", gConfig.wheelSyncVerticalScale);

    gConfig.maximumVerticesPerAtomic = ReadInt("MaximumVerticesPerAtomic", gConfig.maximumVerticesPerAtomic);
    gConfig.collisionSampleMaxAgeFrames = ReadInt("CollisionSampleMaxAgeFrames", gConfig.collisionSampleMaxAgeFrames);

    gConfig.minimumImpulse = std::max(0.0f, gConfig.minimumImpulse);
    gConfig.fullDeformationImpulse = std::max(gConfig.minimumImpulse + 1.0f, gConfig.fullDeformationImpulse);
    gConfig.impulseExponent = Clamp(gConfig.impulseExponent, 0.35f, 2.0f);
    gConfig.playerDeformationScale = std::max(0.0f, gConfig.playerDeformationScale);
    gConfig.nonPlayerDeformationScale = std::max(0.0f, gConfig.nonPlayerDeformationScale);
    gConfig.radius = std::max(0.10f, gConfig.radius);
    gConfig.maximumRadius = std::max(gConfig.radius, gConfig.maximumRadius);
    gConfig.maximumDeformationPerImpact = std::max(0.01f, gConfig.maximumDeformationPerImpact);
    gConfig.maximumTotalDeformation = std::max(gConfig.maximumDeformationPerImpact, gConfig.maximumTotalDeformation);
    gConfig.centerResistance = Clamp(gConfig.centerResistance, 0.0f, 1.0f);
    gConfig.inwardNormalBlend = Clamp(gConfig.inwardNormalBlend, 0.0f, 0.50f);
    gConfig.wheelSyncStrength = Clamp(gConfig.wheelSyncStrength, 0.0f, 1.0f);
    gConfig.wheelSyncRadiusScale = std::max(0.25f, gConfig.wheelSyncRadiusScale);
    gConfig.wheelSyncMaxOffset = std::max(0.0f, gConfig.wheelSyncMaxOffset);
    gConfig.wheelSyncVerticalScale = Clamp(gConfig.wheelSyncVerticalScale, 0.0f, 1.0f);
    gConfig.maximumVerticesPerAtomic = std::max(1, gConfig.maximumVerticesPerAtomic);
    gConfig.collisionSampleMaxAgeFrames = std::max(0, gConfig.collisionSampleMaxAgeFrames);
}

using RwFrameGetLTMFn = RwMatrix* (__cdecl*)(RwFrame*);
using RwFrameUpdateObjectsFn = void (__cdecl*)(RwFrame*);

static RwMatrix* GetFrameLTM(RwFrame* frame) {
    if (!frame)
        return nullptr;
    const auto fn = reinterpret_cast<RwFrameGetLTMFn>(plugin::GetGlobalAddress(kRwFrameGetLTMTarget));
    return fn(frame);
}

static void UpdateFrameObjects(RwFrame* frame) {
    if (!frame)
        return;
    const auto fn = reinterpret_cast<RwFrameUpdateObjectsFn>(plugin::GetGlobalAddress(kRwFrameUpdateObjects));
    fn(frame);
}

static Vec3 WorldToVehiclePoint(const CAutomobile* car, Vec3 world) {
    const Vec3 d = Sub(world, FromC(car->m_matrix.pos));
    return V(Dot(d, FromC(car->m_matrix.right)),
             Dot(d, FromC(car->m_matrix.up)),
             Dot(d, FromC(car->m_matrix.at)));
}

static Vec3 WorldToVehicleVector(const CAutomobile* car, Vec3 world) {
    return V(Dot(world, FromC(car->m_matrix.right)),
             Dot(world, FromC(car->m_matrix.up)),
             Dot(world, FromC(car->m_matrix.at)));
}

static Vec3 VehicleToWorldVector(const CAutomobile* car, Vec3 local) {
    return Add(Add(Mul(FromC(car->m_matrix.right), local.x), Mul(FromC(car->m_matrix.up), local.y)),
               Mul(FromC(car->m_matrix.at), local.z));
}

static Vec3 AtomicToVehiclePoint(const CAutomobile* car, const RwMatrix& atomicLTM, const RwV3d& local) {
    const Vec3 world = Add(Add(Add(Mul(FromRw(atomicLTM.right), local.x), Mul(FromRw(atomicLTM.up), local.y)),
                               Mul(FromRw(atomicLTM.at), local.z)), FromRw(atomicLTM.pos));
    return WorldToVehiclePoint(car, world);
}

struct CollisionSample {
    CVector point{};
    float impulse{};
    unsigned int frame{};
    bool valid{};
};

struct AtomicState {
    RpAtomic* atomic{};
    RpGeometry* geometry{};
    int vertexCount{};
    std::vector<RwV3d> baseVertices;
    std::vector<Vec3> offsets; // atomic-local plastic displacement
    RwSphere baseSphere{};
    bool deformed{};
};

struct WheelState {
    RwFrame* frame{};
    Vec3 offsetVehicle{};
};

struct DeformData {
    CAutomobile* owner{};
    RpClump* cachedClump{};
    int cachedModel{ -1 };
    std::vector<AtomicState> atomics;
    std::vector<WheelState> wheels;
    CollisionSample collision{};
    Vec3 boundsMin{};
    Vec3 boundsMax{};
    bool boundsValid{};
    bool initialized{};

    DeformData() = default;
    explicit DeformData(CAutomobile* vehicle) : owner(vehicle) {}
};

static std::unordered_map<CAutomobile*, DeformData> gVehicleData;

static DeformData* FindData(CAutomobile* car) {
    const auto it = gVehicleData.find(car);
    return it == gVehicleData.end() ? nullptr : &it->second;
}

static DeformData& GetData(CAutomobile* car) {
    auto result = gVehicleData.emplace(car, DeformData(car));
    if (!result.first->second.owner)
        result.first->second.owner = car;
    return result.first->second;
}

static bool IsAutomobile(const CVehicle* vehicle) {
    return vehicle && vehicle->m_nVehicleClass == VEHICLE_AUTOMOBILE;
}

static bool IsEligibleVehicle(CVehicle* vehicle) {
    return gConfig.enabled && IsAutomobile(vehicle) &&
        (!gConfig.onlyPlayerVehicle || vehicle == FindPlayerVehicle());
}

static void IncludeBounds(DeformData& data, Vec3 p) {
    if (!data.boundsValid) {
        data.boundsMin = data.boundsMax = p;
        data.boundsValid = true;
        return;
    }
    data.boundsMin.x = std::min(data.boundsMin.x, p.x);
    data.boundsMin.y = std::min(data.boundsMin.y, p.y);
    data.boundsMin.z = std::min(data.boundsMin.z, p.z);
    data.boundsMax.x = std::max(data.boundsMax.x, p.x);
    data.boundsMax.y = std::max(data.boundsMax.y, p.y);
    data.boundsMax.z = std::max(data.boundsMax.z, p.z);
}

static bool FrameIsOrIsBelow(RwFrame* frame, RwFrame* ancestor) {
    for (RwFrame* f = frame; f; f = RwFrameGetParent(f)) {
        if (f == ancestor)
            return true;
    }
    return false;
}

static bool IsWheelAtomic(const CAutomobile* car, RpAtomic* atomic) {
    if (!car || !atomic)
        return false;
    RwFrame* frame = RpAtomicGetFrame(atomic);
    for (int node = CAR_WHEEL_RF; node <= CAR_WHEEL_LB; ++node) {
        RwFrame* wheel = car->m_aCarNodes[node];
        if (wheel && FrameIsOrIsBelow(frame, wheel))
            return true;
    }
    return false;
}

static RpGeometry* CloneGeometryOwned(const RpGeometry* source) {
    if (!source || source->numVertices <= 0 || source->numTriangles < 0 || source->numMorphTargets <= 0 ||
        !source->morphTarget || !source->morphTarget[0].verts || source->numTexCoordSets < 0 ||
        source->numTexCoordSets > rwMAXTEXTURECOORDS)
        return nullptr;

    RwUInt32 format = source->flags;
    format &= ~(rpGEOMETRYNATIVE | rpGEOMETRYNATIVEINSTANCE);
    format &= ~0x00FF0000u;
    format |= rpGEOMETRYTEXCOORDSETS(source->numTexCoordSets) | rpGEOMETRYPOSITIONS;

    RpGeometry* clone = RpGeometryCreate(source->numVertices, source->numTriangles, format);
    if (!clone)
        return nullptr;

    if (source->numMorphTargets > 1 && RpGeometryAddMorphTargets(clone, source->numMorphTargets - 1) < 0) {
        RpGeometryDestroy(clone);
        return nullptr;
    }
    if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL)) {
        RpGeometryDestroy(clone);
        return nullptr;
    }

    bool ok = true;
    clone->ignoredSurfaceProps = source->ignoredSurfaceProps;

    if (source->numTriangles > 0 && (!source->triangles || !clone->triangles))
        ok = false;

    for (RwInt32 i = 0; ok && i < source->numTriangles; ++i) {
        const RpTriangle& src = source->triangles[i];
        RpTriangle& dst = clone->triangles[i];
        const RwUInt16 a = src.vertIndex[0], b = src.vertIndex[1], c = src.vertIndex[2];
        if (a >= source->numVertices || b >= source->numVertices || c >= source->numVertices ||
            !RpGeometryTriangleSetVertexIndices(clone, &dst, a, b, c)) {
            ok = false;
            break;
        }
        const RwInt32 material = src.matIndex;
        if (material < 0 || material >= source->matList.numMaterials || !source->matList.materials ||
            !source->matList.materials[material] ||
            !RpGeometryTriangleSetMaterial(clone, &dst, source->matList.materials[material])) {
            ok = false;
        }
    }

    if (ok && source->preLitLum) {
        if (!clone->preLitLum)
            ok = false;
        else
            std::memcpy(clone->preLitLum, source->preLitLum,
                sizeof(RwRGBA) * static_cast<size_t>(source->numVertices));
    }

    for (RwInt32 set = 0; ok && set < source->numTexCoordSets; ++set) {
        if (source->texCoords[set] && !clone->texCoords[set]) {
            ok = false;
            break;
        }
        if (source->texCoords[set])
            std::memcpy(clone->texCoords[set], source->texCoords[set],
                sizeof(RwTexCoords) * static_cast<size_t>(source->numVertices));
    }

    for (RwInt32 morph = 0; ok && morph < source->numMorphTargets; ++morph) {
        const RpMorphTarget& src = source->morphTarget[morph];
        RpMorphTarget& dst = clone->morphTarget[morph];
        dst.boundingSphere = src.boundingSphere;
        if (!src.verts || !dst.verts) {
            ok = false;
            break;
        }
        std::memcpy(dst.verts, src.verts, sizeof(RwV3d) * static_cast<size_t>(source->numVertices));
        if (src.normals && !dst.normals) {
            ok = false;
            break;
        }
        if (src.normals)
            std::memcpy(dst.normals, src.normals, sizeof(RwV3d) * static_cast<size_t>(source->numVertices));
    }

    if (!RpGeometryUnlock(clone))
        ok = false;
    if (!ok) {
        RpGeometryDestroy(clone);
        return nullptr;
    }
    return clone;
}

static Vec3 WorldVectorToAtomic(const RwMatrix& ltm, Vec3 world) {
    return V(Dot(world, FromRw(ltm.right)), Dot(world, FromRw(ltm.up)), Dot(world, FromRw(ltm.at)));
}

static bool CacheAtomic(DeformData& data, CAutomobile* car, RpAtomic* atomic) {
    if (!atomic || !atomic->geometry || IsWheelAtomic(car, atomic))
        return false;

    RpGeometry* source = atomic->geometry;
    if (!(source->flags & rpGEOMETRYPOSITIONS) || source->numVertices <= 0 || source->numMorphTargets <= 0 ||
        !source->morphTarget || !source->morphTarget[0].verts)
        return false;

    RwFrame* frame = RpAtomicGetFrame(atomic);
    RwMatrix* ltm = GetFrameLTM(frame);
    if (!ltm)
        return false;

    RpGeometry* clone = CloneGeometryOwned(source);
    if (!clone)
        return false;
    if (!RpAtomicSetGeometry(atomic, clone, 0) || atomic->geometry != clone) {
        RpGeometryDestroy(clone);
        return false;
    }
    RpGeometryDestroy(clone); // atomic owns the remaining reference

    AtomicState state;
    state.atomic = atomic;
    state.geometry = atomic->geometry;
    state.vertexCount = std::min(state.geometry->numVertices, gConfig.maximumVerticesPerAtomic);
    if (state.vertexCount <= 0 || !state.geometry->morphTarget || !state.geometry->morphTarget[0].verts)
        return false;

    RpMorphTarget& target = state.geometry->morphTarget[0];
    state.baseSphere = target.boundingSphere;
    state.baseVertices.assign(target.verts, target.verts + state.vertexCount);
    state.offsets.assign(static_cast<size_t>(state.vertexCount), Vec3{});

    // The collision model is the preferred GTA III body envelope. Geometry bounds are
    // only a fallback for unusual models without a collision model.
    if (!data.boundsValid) {
        for (const RwV3d& vertex : state.baseVertices)
            IncludeBounds(data, AtomicToVehiclePoint(car, *ltm, vertex));
    }
    data.atomics.emplace_back(std::move(state));
    return true;
}

static RpAtomic* CacheAtomicCallback(RpAtomic* atomic, void* context) {
    auto* data = static_cast<DeformData*>(context);
    if (data && IsAutomobile(data->owner))
        CacheAtomic(*data, reinterpret_cast<CAutomobile*>(data->owner), atomic);
    return atomic;
}

static void CaptureWheels(CAutomobile* car, DeformData& data) {
    data.wheels.clear();
    for (int node = CAR_WHEEL_RF; node <= CAR_WHEEL_LB; ++node) {
        RwFrame* frame = car->m_aCarNodes[node];
        RwMatrix* ltm = GetFrameLTM(frame);
        if (!frame || !ltm)
            continue;

        bool duplicate = false;
        for (const WheelState& wheel : data.wheels) {
            if (wheel.frame == frame) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            WheelState wheel;
            wheel.frame = frame;
            data.wheels.push_back(wheel);
        }
    }
}

static void ResetData(DeformData& data) {
    data.cachedClump = nullptr;
    data.cachedModel = -1;
    data.atomics.clear();
    data.wheels.clear();
    data.collision = {};
    data.boundsValid = false;
    data.initialized = false;
}

static bool EnsureCache(CAutomobile* car, DeformData& data) {
    if (!IsEligibleVehicle(car) || !car->m_pRwClump)
        return false;

    if (data.initialized && (data.cachedClump != car->m_pRwClump || data.cachedModel != car->m_nModelIndex))
        ResetData(data);
    if (data.initialized)
        return !data.atomics.empty();

    data.initialized = true;
    data.cachedClump = car->m_pRwClump;
    data.cachedModel = car->m_nModelIndex;

    if (CColModel* col = car->GetColModel()) {
        data.boundsMin = FromC(col->m_boundBox.m_vecMin);
        data.boundsMax = FromC(col->m_boundBox.m_vecMax);
        data.boundsValid = true;
    }

    CaptureWheels(car, data);
    RpClumpForAllAtomics(car->m_pRwClump, CacheAtomicCallback, &data);
    return !data.atomics.empty();
}

static void RecordCollisionCandidate(CPhysical* physical, const CColPoint& point, float impulse) {
    if (!physical || physical->m_nType != ENTITY_TYPE_VEHICLE || !std::isfinite(impulse) || impulse <= 0.0f)
        return;

    auto* vehicle = reinterpret_cast<CVehicle*>(physical);
    if (!IsAutomobile(vehicle))
        return;

    auto* car = reinterpret_cast<CAutomobile*>(vehicle);
    DeformData& data = GetData(car);
    CollisionSample& sample = data.collision;
    const unsigned int frame = CTimer::m_FrameCounter;

    // GTA III's ProcessCollisionSectorList does the same thing for the authoritative
    // damage record: only the collision result whose impulse beats the current best
    // becomes m_fDamageImpulse/m_nDamagePieceType/m_vecDamageNormal. Mirror that rule
    // for the render-space contact point instead of retaining an arbitrary last contact.
    if (!sample.valid || sample.frame != frame || impulse > sample.impulse) {
        sample.point = point.m_vecPoint;
        sample.impulse = impulse;
        sample.frame = frame;
        sample.valid = true;
    }
}

static int ComponentToNode(unsigned short component) {
    switch (static_cast<tComponent>(component)) {
    case COMPONENT_WHEEL_LF: return CAR_WHEEL_LF;
    case COMPONENT_WHEEL_RF: return CAR_WHEEL_RF;
    case COMPONENT_WHEEL_LR: return CAR_WHEEL_LB;
    case COMPONENT_WHEEL_RR: return CAR_WHEEL_RB;
    case COMPONENT_BONNET: return CAR_BONNET;
    case COMPONENT_BOOT: return CAR_BOOT;
    case COMPONENT_DOOR_LF: return CAR_DOOR_LF;
    case COMPONENT_DOOR_RF: return CAR_DOOR_RF;
    case COMPONENT_DOOR_LR: return CAR_DOOR_LR;
    case COMPONENT_DOOR_RR: return CAR_DOOR_RR;
    case COMPONENT_WING_LF: return CAR_WING_LF;
    case COMPONENT_WING_RF: return CAR_WING_RF;
    case COMPONENT_WING_LR: return CAR_WING_LR;
    case COMPONENT_WING_RR: return CAR_WING_RR;
    case COMPONENT_WINDSCREEN: return CAR_WINDSCREEN;
    case COMPONENT_BUMP_FRONT: return CAR_BUMP_FRONT;
    case COMPONENT_BUMP_REAR: return CAR_BUMP_REAR;
    default: return CAR_NODE_NONE;
    }
}

static CollisionSample GetImpact(CAutomobile* car, const DeformData& data, unsigned short component) {
    if (data.collision.valid) {
        const unsigned int age = CTimer::m_FrameCounter - data.collision.frame;
        if (age <= static_cast<unsigned int>(gConfig.collisionSampleMaxAgeFrames))
            return data.collision;
    }

    CollisionSample fallback;
    fallback.point = car->m_matrix.pos;
    const int node = ComponentToNode(component);
    if (node > CAR_NODE_NONE && node < CAR_NUM_NODES) {
        RwMatrix* ltm = GetFrameLTM(car->m_aCarNodes[node]);
        if (ltm)
            fallback.point = CVector(ltm->pos.x, ltm->pos.y, ltm->pos.z);
    }
    fallback.impulse = car->m_fDamageImpulse;
    fallback.frame = CTimer::m_FrameCounter;
    fallback.valid = true;
    return fallback;
}

struct DentField {
    Vec3 impact{};
    Vec3 push{};
    float radius{};
    float peakDepth{};
};

static float CenterFlex(const DeformData& data, Vec3 p) {
    if (!data.boundsValid || gConfig.centerResistance <= 0.0f)
        return 1.0f;

    const Vec3 center = Mul(Add(data.boundsMin, data.boundsMax), 0.5f);
    const Vec3 half = Mul(Sub(data.boundsMax, data.boundsMin), 0.5f);
    const float nx = std::fabs(p.x - center.x) / std::max(half.x, 0.10f);
    const float ny = std::fabs(p.y - center.y) / std::max(half.y, 0.10f);
    const float nz = std::fabs(p.z - center.z) / std::max(half.z, 0.10f);
    const float edge = Clamp(std::max(nx, std::max(ny, nz)), 0.0f, 1.0f);
    const float core = 1.0f - edge;
    return 1.0f - gConfig.centerResistance * core;
}

static float DentWeight(Vec3 restVehicle, const DentField& field, float radiusScale = 1.0f) {
    const float radius = field.radius * radiusScale;
    if (radius <= kEpsilon)
        return 0.0f;

    // Late III test branches converged on this exact rule: measure every new hit
    // from the immutable/original vertex, not from base+existingOffset.  That keeps
    // repeated impacts plastic without letting the active dent region chase itself.
    const float distance = Length(Sub(restVehicle, field.impact));
    if (distance >= radius)
        return 0.0f;
    return Clamp(1.0f - distance / radius, 0.0f, 1.0f);
}

static Vec3 DentDelta(const DeformData& data, Vec3 restVehicle, const DentField& field,
                      float radiusScale = 1.0f, float strengthScale = 1.0f) {
    const float weight = DentWeight(restVehicle, field, radiusScale);
    if (weight <= 0.0f)
        return {};

    const float amount = field.peakDepth * weight * CenterFlex(data, restVehicle) * strengthScale;
    return Mul(field.push, amount);
}

static void ClampVector(Vec3& v, float maximum) {
    const float len = Length(v);
    if (len > maximum && len > kEpsilon)
        v = Mul(v, maximum / len);
}

static bool ApplyDentToAtomic(CAutomobile* car, DeformData& data, AtomicState& state,
                              const DentField& field) {
    if (!car || !state.atomic || !state.geometry || state.atomic->geometry != state.geometry ||
        state.vertexCount <= 0 || state.baseVertices.size() != static_cast<size_t>(state.vertexCount) ||
        state.offsets.size() != static_cast<size_t>(state.vertexCount) || !state.geometry->morphTarget ||
        !state.geometry->morphTarget[0].verts)
        return false;

    RwMatrix* ltm = GetFrameLTM(RpAtomicGetFrame(state.atomic));
    if (!ltm)
        return false;

    bool changed = false;
    float maxOffset = 0.0f;

    for (int i = 0; i < state.vertexCount; ++i) {
        const size_t index = static_cast<size_t>(i);
        Vec3& localOffset = state.offsets[index];
        const RwV3d& base = state.baseVertices[index];

        // The rest vertex is immutable, but its component frame is not.  Transform the
        // rest point through the CURRENT frame so an open door/bonnet/boot is hit where
        // GTA III currently renders it, while the old plastic offset stays local to that
        // component and therefore follows subsequent animation naturally.
        const Vec3 restVehicle = AtomicToVehiclePoint(car, *ltm, base);
        const Vec3 vehicleDelta = DentDelta(data, restVehicle, field);
        if (LengthSq(vehicleDelta) > kEpsilon * kEpsilon) {
            const Vec3 worldDelta = VehicleToWorldVector(car, vehicleDelta);
            const Vec3 localDelta = WorldVectorToAtomic(*ltm, worldDelta);
            localOffset = Add(localOffset, localDelta);
            ClampVector(localOffset, gConfig.maximumTotalDeformation);
            changed = true;
        }
        maxOffset = std::max(maxOffset, Length(localOffset));
    }

    if (!changed || !RpGeometryLock(state.geometry, rpGEOMETRYLOCKVERTICES))
        return false;

    RpMorphTarget& target = state.geometry->morphTarget[0];
    for (int i = 0; i < state.vertexCount; ++i) {
        const size_t index = static_cast<size_t>(i);
        const RwV3d& base = state.baseVertices[index];
        const Vec3& offset = state.offsets[index];
        target.verts[i] = { base.x + offset.x, base.y + offset.y, base.z + offset.z };
    }

    target.boundingSphere = state.baseSphere;
    target.boundingSphere.radius += maxOffset + 0.02f;
    if (!RpGeometryUnlock(state.geometry))
        return false;
    state.deformed = true;
    return true;
}

static void ApplyDentToWheels(CAutomobile* car, DeformData& data, const DentField& field) {
    if (!car || !gConfig.wheelSync)
        return;

    for (WheelState& wheel : data.wheels) {
        RwMatrix* ltm = GetFrameLTM(wheel.frame);
        if (!ltm)
            continue;
        const Vec3 current = WorldToVehiclePoint(car, FromRw(ltm->pos));
        Vec3 delta = DentDelta(data, current, field,
                               gConfig.wheelSyncRadiusScale, gConfig.wheelSyncStrength);
        delta.z *= gConfig.wheelSyncVerticalScale;
        wheel.offsetVehicle = Add(wheel.offsetVehicle, delta);
        ClampVector(wheel.offsetVehicle, gConfig.wheelSyncMaxOffset);
    }
}

static DentField MakeDentField(CAutomobile* car, const DeformData& data,
                               const CollisionSample& impact, float impulse, float scale) {
    DentField field;
    field.impact = WorldToVehiclePoint(car, FromC(impact.point));

    const Vec3 center = data.boundsValid
        ? Mul(Add(data.boundsMin, data.boundsMax), 0.5f)
        : V(0.0f, 0.0f, 0.0f);
    const Vec3 inward = Normalize(Sub(center, field.impact), V(0.0f, -1.0f, 0.0f));

    // GTA III stores m_vecDamageNormal with the same winning damage record as
    // m_fDamageImpulse/m_nDamagePieceType.  Use that authoritative direction, only
    // flipping the sign when necessary so the visual dent cannot extrude outward.
    Vec3 damageNormal = Normalize(WorldToVehicleVector(car, FromC(car->m_vecDamageNormal)), inward);
    if (Dot(damageNormal, inward) < 0.0f)
        damageNormal = Mul(damageNormal, -1.0f);
    field.push = Normalize(Add(Mul(damageNormal, 1.0f - gConfig.inwardNormalBlend),
                               Mul(inward, gConfig.inwardNormalBlend)), inward);

    const float range = gConfig.fullDeformationImpulse - gConfig.minimumImpulse;
    const float normalized = Clamp((impulse - gConfig.minimumImpulse) / range, 0.0f, 1.0f);
    const float severity = Clamp(std::pow(normalized, gConfig.impulseExponent) * scale, 0.0f, 1.0f);

    // VC's working deformation mod grows a spherical radius with impact strength.  The
    // old III direct-field branches independently converged on the same broad idea.
    // Keep III's raw-impulse severity, but use that simple spherical shape instead of
    // the refactor's axial capsule, which could reject the visible shell entirely.
    field.peakDepth = gConfig.maximumDeformationPerImpact * severity;
    field.radius = gConfig.radius +
        (gConfig.maximumRadius - gConfig.radius) * std::sqrt(severity);
    return field;
}

static void DeformVehicle(CAutomobile* car, float impulse, unsigned short component) {
    if (!IsEligibleVehicle(car) || !std::isfinite(impulse) || impulse < gConfig.minimumImpulse)
        return;

    DeformData& data = GetData(car);
    if (!EnsureCache(car, data))
        return;

    const CollisionSample impact = GetImpact(car, data, component);
    const float scale = car == FindPlayerVehicle()
        ? gConfig.playerDeformationScale
        : gConfig.nonPlayerDeformationScale;
    const DentField field = MakeDentField(car, data, impact, impulse, scale);
    if (field.peakDepth <= kEpsilon)
        return;

    for (AtomicState& state : data.atomics)
        ApplyDentToAtomic(car, data, state, field);
    ApplyDentToWheels(car, data, field);
}

struct WheelRenderBackup {
    RwFrame* frame{};
    RwV3d position{};
};

static void ApplyWheelRenderOffsets(CAutomobile* car, std::vector<WheelRenderBackup>& backups) {
    if (!car || !gConfig.wheelSync)
        return;

    DeformData* data = FindData(car);
    if (!data || !data->initialized)
        return;

    backups.reserve(data->wheels.size());
    for (WheelState& wheel : data->wheels) {
        if (!wheel.frame || LengthSq(wheel.offsetVehicle) <= kEpsilon * kEpsilon)
            continue;

        backups.push_back({ wheel.frame, wheel.frame->modelling.pos });
        const Vec3 worldDelta = VehicleToWorldVector(car, wheel.offsetVehicle);
        Vec3 parentDelta = worldDelta;
        if (RwFrame* parent = RwFrameGetParent(wheel.frame)) {
            if (RwMatrix* parentLTM = GetFrameLTM(parent))
                parentDelta = WorldVectorToAtomic(*parentLTM, worldDelta);
        }

        wheel.frame->modelling.pos.x += parentDelta.x;
        wheel.frame->modelling.pos.y += parentDelta.y;
        wheel.frame->modelling.pos.z += parentDelta.z;
        UpdateFrameObjects(wheel.frame);
    }
}

static void RestoreWheelRenderOffsets(std::vector<WheelRenderBackup>& backups) {
    for (auto it = backups.rbegin(); it != backups.rend(); ++it) {
        if (!it->frame)
            continue;
        it->frame->modelling.pos = it->position;
        UpdateFrameObjects(it->frame);
    }
}

static void RestoreVehicle(CAutomobile* car) {
    DeformData* data = car ? FindData(car) : nullptr;
    if (!data)
        return;

    for (AtomicState& state : data->atomics) {
        std::fill(state.offsets.begin(), state.offsets.end(), Vec3{});
        if (!state.deformed || !state.atomic || !state.geometry || state.atomic->geometry != state.geometry ||
            !state.geometry->morphTarget || !state.geometry->morphTarget[0].verts)
            continue;

        if (!RpGeometryLock(state.geometry, rpGEOMETRYLOCKVERTICES))
            continue;
        RpMorphTarget& target = state.geometry->morphTarget[0];
        std::memcpy(target.verts, state.baseVertices.data(), sizeof(RwV3d) * state.baseVertices.size());
        target.boundingSphere = state.baseSphere;
        RpGeometryUnlock(state.geometry);
        state.deformed = false;
    }

    for (WheelState& wheel : data->wheels)
        wheel.offsetVehicle = {};
    data->collision = {};
}

using ApplyCollisionFn = bool(__thiscall*)(CPhysical*, CPhysical*, CColPoint&, float*, float*);
using ApplyCollisionAltFn = bool(__thiscall*)(CPhysical*, CEntity*, CColPoint&, float*, CVector&, CVector&);
using VehicleDamageFn = void(__thiscall*)(CAutomobile*, float, unsigned short);
using FixFn = void(__thiscall*)(CAutomobile*);
using EntityRenderFn = void(__thiscall*)(CEntity*);

static bool __fastcall ApplyCollisionHook(CPhysical* self, void*, CPhysical* other, CColPoint& point,
                                          float* impulseA, float* impulseB) {
    const auto original = reinterpret_cast<ApplyCollisionFn>(plugin::GetGlobalAddress(kApplyCollisionTarget));
    const bool result = original(self, other, point, impulseA, impulseB);
    if (result) {
        if (impulseA)
            RecordCollisionCandidate(self, point, *impulseA);
        if (impulseB)
            RecordCollisionCandidate(other, point, *impulseB);
    }
    return result;
}

static bool __fastcall ApplyCollisionAltHook(CPhysical* self, void*, CEntity* other, CColPoint& point,
                                             float* impulse, CVector& moveSpeed, CVector& turnSpeed) {
    const auto original = reinterpret_cast<ApplyCollisionAltFn>(plugin::GetGlobalAddress(kApplyCollisionAltTarget));
    const bool result = original(self, other, point, impulse, moveSpeed, turnSpeed);
    if (result && impulse)
        RecordCollisionCandidate(self, point, *impulse);
    return result;
}

static void __fastcall VehicleDamageHook(CAutomobile* car, void*, float impulse, unsigned short component) {
    float usedImpulse = impulse;
    unsigned short usedComponent = component;
    if (car && (!std::isfinite(usedImpulse) || usedImpulse == 0.0f)) {
        usedImpulse = car->m_fDamageImpulse;
        usedComponent = static_cast<unsigned short>(car->m_nDamagePieceType);
    }

    if (car)
        DeformVehicle(car, usedImpulse, usedComponent);

    const auto original = reinterpret_cast<VehicleDamageFn>(plugin::GetGlobalAddress(kVehicleDamageTarget));
    original(car, impulse, component);
}

static void __fastcall FixHook(CAutomobile* car, void*) {
    RestoreVehicle(car);
    const auto original = reinterpret_cast<FixFn>(plugin::GetGlobalAddress(kFixTarget));
    original(car);
}

static void __fastcall AutomobileEntityRenderHook(CAutomobile* car, void*) {
    std::vector<WheelRenderBackup> backups;
    ApplyWheelRenderOffsets(car, backups);

    const auto original = reinterpret_cast<EntityRenderFn>(plugin::GetGlobalAddress(kEntityRenderTarget));
    original(car);

    RestoreWheelRenderOffsets(backups);
}

static void InstallHooks() {
    static constexpr uintptr_t applyCollisionCalls[] = {
        0x49C76A, 0x49CB0A, 0x49CF92, 0x49D39F,
        0x49E949, 0x49EC7F, 0x49F089, 0x49F3FD
    };
    static constexpr uintptr_t applyCollisionAltCalls[] = { 0x49BE36, 0x49C0D2 };

    for (uintptr_t address : applyCollisionCalls)
        patch::RedirectCall(address, ApplyCollisionHook);
    for (uintptr_t address : applyCollisionAltCalls)
        patch::RedirectCall(address, ApplyCollisionAltHook);
    patch::RedirectCall(kVehicleDamageCall, VehicleDamageHook);
    patch::RedirectCall(kFixCall, FixHook);

    if (gConfig.wheelSync)
        patch::RedirectCall(kAutomobileEntityRenderCall, AutomobileEntityRenderHook);
}

static void OnVehicleSetModel(CVehicle* vehicle, int) {
    if (!IsAutomobile(vehicle))
        return;
    gVehicleData.erase(reinterpret_cast<CAutomobile*>(vehicle));
}

static void OnVehicleDtor(CVehicle* vehicle) {
    if (!IsAutomobile(vehicle))
        return;
    // The SDK event fires before CVehicle destruction, while the clump still exists.
    // The atomics own their private geometry references; erasing our raw state here is
    // enough and avoids touching RenderWare objects during/after their destruction.
    gVehicleData.erase(reinterpret_cast<CAutomobile*>(vehicle));
}

class Plugin {
public:
    Plugin() {
        LoadConfig();
        Events::vehicleSetModelEvent.after += OnVehicleSetModel;
        Events::vehicleDtorEvent += OnVehicleDtor;
        if (gConfig.enabled)
            InstallHooks();
    }
};

static Plugin gPlugin;

} // namespace VehDeformIII
