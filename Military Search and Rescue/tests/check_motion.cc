// Integration check: query actual ECS poses after Gazebo Physics updates.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Physics.hh>

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: check_motion WORLD.sdf EXPECTED_MOVER_COUNT\n";
    return 2;
  }
  std::map<std::string, gz::math::Pose3d> first, current, beforePause;
  std::map<std::string, double> distance;
  gz::sim::TestFixture fixture(argv[1]);
  fixture.OnPostUpdate([&](const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm)
  {
    if (!info.paused && info.iterations % 2000 == 0)
      std::cout << "Simulated "
                << std::chrono::duration<double>(info.simTime).count()
                << " s" << std::endl;
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
             gz::sim::components::Pose>([&](const gz::sim::Entity &,
             const auto *, const auto *name, const auto *pose)
    {
      const auto &n = name->Data();
      if (n.find("SAR_DynamicPerson_") != 0 &&
          n.find("SAR_CarriedStretcher_") != 0) return true;
      first.emplace(n, pose->Data());
      current[n] = pose->Data();
      distance[n] = std::max(distance[n],
          first[n].Pos().Distance(current[n].Pos()));
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
  sdf::Root description;
  if (!description.Load(argv[1]).empty()) return 1;
  // A world without <physics> still parses; sdformat's default profile is
  // what Gazebo would use, so fall back to its step rather than to a null.
  double step = 0.001;
  const auto *world = description.WorldByIndex(0);
  if (world && world->PhysicsCount() > 0 && world->PhysicsByIndex(0))
    step = world->PhysicsByIndex(0)->MaxStepSize();
  if (!(step > 0))
  {
    std::cerr << "Non-positive max_step_size in " << argv[1] << "\n";
    return 1;
  }
  if (!server->Run(true, static_cast<uint64_t>(std::ceil(16.0 / step)), false)) return 1;
  bool ok = current.size() == static_cast<size_t>(std::stoi(argv[2]));
  for (const auto &[name, d] : distance)
  {
    std::cout << name << " moved " << d << " m\n";
    ok = ok && d > 0.5;
  }
  beforePause = current;
  // Paused Run(N) never reaches N simulation iterations; explicitly step ECS.
  for (int i = 0; i < 20; ++i)
    if (!server->RunOnce(true)) return 1;
  for (const auto &[name, pose] : current)
    ok = ok && pose == beforePause.at(name);
  std::cout << (ok ? "PASS" : "FAIL") << ": motion and pause checks\n";
  return ok ? 0 : 1;
}
