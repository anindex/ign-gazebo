\page migrationlinkapi

# Migration from Gazebo-classic: Link API

When migrating plugins from Gazebo-classic to Ignition Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::phyiscs::Link](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html)
class.

If you're trying to use some API which doesn't have an equivalent on Ignition
yet, feel free to
[ticket an issue](https://github.com/ignitionrobotics/ign-gazebo/issues/).

## Link API

Gazebo-classic's `gazebo::physics::Link` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Link::GetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a9a98946a64f3893b085f650932c9dfee) / [Link::SetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a5d74ac4d7a230aed1ab4b11933b16e92)
* **Read family**: Getting children and parent
    * Example: [Link::GetCollision](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html#ae52be77915eb1e972e7571a20e4ab562)
* **Write family**: Adding children, changing parent
    * Example: [Link::RemoveChildren](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#aa85d2386e6fb02bdbec060a74b63238a)
* **Lifecycle**: Functions to control the link's lifecycle
    * Example: [Link::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html#ae048ef824aaf614707c1496a2aefd415)
* **Others**: Functions that don't fit any of the categories above
    * Example: [Link::PlaceOnEntity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a9ecbfeb56940cacd75f55bed6aa9fcb4)

You'll find the Ignition APIs below on the following headers:

* [ignition/gazebo/Link.hh](https://ignitionrobotics.org/api/gazebo/3.3/Link_8hh.html)
* [ignition/gazebo/Util.hh](https://ignitionrobotics.org/api/gazebo/3.3/Util_8hh.html)
* [ignition/gazebo/SdfEntityCreator.hh](https://ignitionrobotics.org/api/gazebo/3.3/SdfEntityCreator_8hh.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Link API is related to setting and getting
properties. These functions are great candidates to have equivalents on Ignition
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as links.

---

Classic | Ignition
-- | --
AddForce | TODO
AddForceAtRelativePosition | TODO
AddForceAtWorldPosition | TODO
AddLinkForce | TODO
AddRelativeForce | TODO
AddRelativeTorque | TODO
AddTorque | TODO
AddType | `ecm.CreateComponent<Type>(entity, Type())`
Battery |
BoundingBox |
CollisionBoundingBox |
DirtyPose | Not supported
FillMsg | TODO
GetAngularDamping | TODO
GetEnabled |
GetGravityMode | TODO
GetId | `ignition::gazebo::Link::Entity`
GetInertial |
GetKinematic | TODO
GetLinearDamping | TODO
GetName | `ignition::gazebo::Link::Name`
GetSDF |
GetSDFDom |
GetSaveable | Not supported
GetScopedName | `ignition::gazebo::scopedName`
GetSelfCollide | `ignition::gazebo::Link::SelfCollide`
GetSensorName |
GetType | `ignition::gazebo::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | TODO
GetWorldEnergyPotential | TODO
HasType |  `ignition::gazebo::isType`
InitialRelativePose |
IsCanonicalLink | `ignition::gazebo::Link::IsCanonical`
IsSelected | Selection is client-specific, not porting
IsStatic | `ignition::gazebo::Link::Static`
MoveFrame |
Print |
ProcessMsg |
RelativeAngularAccel |
RelativeAngularVel |
RelativeForce | TODO
RelativeLinearAccel |
RelativeLinearVel |
RelativePose |
RelativeTorque |
RelativeWindLinearVel |
SDFPoseRelativeToParent |
SDFSemanticPose |
SetAngularDamping |
SetAngularVel |
SetAnimation |
SetAnimation |
SetAutoDisable |
SetCanonicalLink |
SetCollideMode |
SetEnabled |
SetForce | TODO
SetGravityMode |
SetInertial |
SetInitialRelativePose |
SetKinematic |
SetLaserRetro |
SetLinearDamping |
SetLinearVel |
SetLinkStatic |
SetName |
SetPublishData |
SetRelativePose |
SetSaveable | Not supported
SetScale |
SetSelected |  Selection is client-specific, not porting
SetSelfCollide |
SetState |
SetStatic |
SetTorque | TODO
SetVisualPose |
SetWindEnabled |
SetWindMode |
SetWorldPose |
SetWorldTwist |
StopAnimation |
TypeStr | `ignition::gazebo::entityTypeStr`
URI |
UpdateParameters |
VisualPose |
WindMode | `ignition::gazebo::Link::WindMode`
WorldAngularAccel |
WorldAngularMomentum |
WorldAngularVel |
WorldCoGLinearVel |
WorldCoGPose |
WorldForce | TODO
WorldInertiaMatrix |
WorldInertialPose |
WorldLinearAccel |
WorldLinearVel |
WorldPose |  `ignition::gazebo::worldPose`
WorldTorque | TODO
WorldWindLinearVel |

---

## Read family

These APIs deal with reading information related to child / parent
relationships.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Ignition,
they deal with entity IDs.

---

Classic | Ignition
-- | --
BatteryCount |
FindAllConnectedLinksHelper |
GetByName | Use type-specific `ignition::gazebo::Link::*ByName`
GetChild | Use type-specific `ignition::gazebo::Link::*ByName`
GetChildCollision | `ignition::gazebo::Link::CollisionByName`
GetChildCount | Use type-specific `ignition::gazebo::Link::*Count`
GetChildJoint |  `ignition::gazebo::Link::ChildJointByName`
GetChildJointsLinks | See joint API
GetChildLink | Not supported
GetCollision | `ignition::gazebo::Link::CollisionByName`
GetCollisions | `ignition::gazebo::Link::Collisions`
GetModel | `ignition::gazebo::Link::Parent`
GetParent | `ignition::gazebo::Link::Parent`
GetParentId | `ignition::gazebo::Link::Parent`
GetParentJoints | `ignition::gazebo::Link::ParentJointByName`
GetParentJointsLinks | See joint API
GetParentModel | `ignition::gazebo::Link::Parent`
GetSensorCount | `ignition::gazebo::Link::SensorCount`
GetVisualMessage | See visual API
GetWorld |  `ignition::gazebo::worldEntity`
VisualId | `ignition::gazebo::Link::VisualByName`
Visuals | `ignition::gazebo::Link::Visuals`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Ignition
-- | --
AddChild | TODO
AddChildJoint | TODO
AddParentJoint | TODO
AttachStaticLink | TODO
AttachStaticModel | TODO
CreateJoint | TODO
CreateLink | TODO
DetachAllStaticModels | TODO
DetachStaticLink | TODO
DetachStaticModel | TODO
RemoveChild | TODO
RemoveChildJoint | TODO
RemoveChildren | TODO
RemoveCollision | TODO
RemoveJoint | TODO
RemoveParentJoint | TODO
SetCanonicalLink | TODO
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions aren't related to the state of a link, but perform some
processing related to the link's lifecycle, like initializing, updating or
terminating it.

---

Classic | Ignition
-- | --
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadJoints | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadPlugins | TODO
Reset | TODO
ResetPhysicsStates | TODO
Update | Entities are updated by systems
UpdateMass | Entities are updated by systems
UpdateSurface | Entities are updated by systems
UpdateWind | Entities are updated by systems
OnPoseChange | TODO

---

## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

---

Classic | Ignition
-- | --
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Involves Requires a system
PlaceOnNearestEntityBelow | Requires a system

---
