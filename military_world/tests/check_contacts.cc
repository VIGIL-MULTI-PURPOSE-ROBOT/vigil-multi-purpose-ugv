#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ContactSensorData.hh>

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const std::string scenario = argv[2];
  gz::sim::TestFixture fixture(argv[1]);
  double maxA = -4, minB = 4, minSeparation = 8, minZ = 100, maxTilt = 0;
  double xAtOne = -4, xAtTwo = -4;
  unsigned int contacts = 0;
  std::map<std::string, gz::math::Pose3d> poses;
  fixture.OnPostUpdate([&](const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm)
  {
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
             gz::sim::components::Pose>([&](const gz::sim::Entity &, const auto *,
              const auto *name, const auto *pose)
    {
      if (name->Data().find("SAR_DynamicPerson_test_") != 0) return true;
      poses[name->Data()] = pose->Data();
      minZ = std::min(minZ, pose->Data().Z());
      maxTilt = std::max(maxTilt, std::hypot(pose->Data().Rot().Roll(), pose->Data().Rot().Pitch()));
      return true;
    });
    // find(), not operator[]: the latter inserts a default-constructed pose
    // for a person this scenario never spawns, and every later read of it is
    // a measurement of something that does not exist.
    const auto itA = poses.find("SAR_DynamicPerson_test_a");
    if (itA == poses.end()) return;
    const auto &a = itA->second;
    maxA = std::max(maxA, a.X());
    const auto itB = poses.find("SAR_DynamicPerson_test_b");
    if (itB != poses.end())
    {
      const auto &b = itB->second;
      minB = std::min(minB, b.X());
      minSeparation = std::min(minSeparation, b.X() - a.X());
    }
    const double time = std::chrono::duration<double>(info.simTime).count();
    if (std::abs(time - 1.0) < 0.005) xAtOne = a.X();
    if (std::abs(time - 2.0) < 0.005) xAtTwo = a.X();
    ecm.Each<gz::sim::components::ContactSensorData>([&](const gz::sim::Entity &, const auto *data)
    {
      for (const auto &contact : data->Data().contact())
      {
        const auto pair = contact.collision1().name() + contact.collision2().name();
        if (pair.find("test_wall") != std::string::npos ||
            (pair.find("test_a") != std::string::npos && pair.find("test_b") != std::string::npos))
          ++contacts;
      }
      return true;
    });
  }).Finalize();
  auto server = fixture.Server();
  if (!server)
  {
    std::cerr << "Gazebo refused to build a server for " << argv[1] << "\n";
    return 1;
  }
  server->SetUpdatePeriod(std::chrono::steady_clock::duration::zero());
  if (!server->Run(true, 800, false)) return 1;
  if (poses.find("SAR_DynamicPerson_test_a") == poses.end())
  {
    std::cerr << "No test people were spawned by " << argv[1] << "\n";
    return 1;
  }
  bool ok = minZ > -0.15 && maxTilt < 0.4;
  if (scenario == "free") ok = ok && maxA > 3 && xAtTwo - xAtOne > 1.7;
  if (scenario == "wall") ok = ok && maxA < -0.15 && maxA > -1 && contacts > 0;
  if (scenario == "people") ok = ok && minSeparation > 0.25 && contacts > 0;
  const auto before = poses;
  for (int i = 0; i < 20; ++i)
    if (!server->RunOnce(true)) return 1;
  ok = ok && before == poses;
  std::cout << "scenario=" << scenario << " max_a_x=" << maxA << " min_b_x=" << minB
            << " min_separation=" << minSeparation << " min_z=" << minZ
            << " max_tilt=" << maxTilt << " contacts=" << contacts
            << " cruise_mps=" << xAtTwo - xAtOne << "\n"
            << (ok ? "PASS" : "FAIL") << ": physical contact, speed and pause\n";
  return ok ? 0 : 1;
}
