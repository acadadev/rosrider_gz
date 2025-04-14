// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "rosrider_gz_gazebo/FullSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    rosrider_gz_gazebo::FullSystem,
    gz::sim::System,
    rosrider_gz_gazebo::FullSystem::ISystemConfigure,
    rosrider_gz_gazebo::FullSystem::ISystemPreUpdate,
    rosrider_gz_gazebo::FullSystem::ISystemUpdate,
    rosrider_gz_gazebo::FullSystem::ISystemPostUpdate,
    rosrider_gz_gazebo::FullSystem::ISystemReset
)

namespace rosrider_gz_gazebo 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  gzdbg << "rosrider_gz_gazebo::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "rosrider_gz_gazebo::FullSystem::PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "rosrider_gz_gazebo::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "rosrider_gz_gazebo::FullSystem::PostUpdate" << std::endl;
  }
}

void FullSystem::Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  gzdbg << "rosrider_gz_gazebo::FullSystem::Reset" << std::endl;
}
}  // namespace rosrider_gz_gazebo
