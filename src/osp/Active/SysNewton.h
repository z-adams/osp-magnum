#pragma once

#include <cstdint>

#include "../Resource/PrototypePart.h"

#include "../types.h"
#include "activetypes.h"
#include <Magnum/Math/Vector4.h>

class NewtonBody;
class NewtonCollision;
class NewtonWorld;

namespace osp::active
{

class ActiveScene;

struct DataPhyRigidBody
{
    float m_mass;
    Vector3 m_velocity;
    Vector3 m_rotVelocity;

    Vector3 m_intertia;
    Vector3 m_netForce;
    Vector3 m_netTorque;
};

struct ACompNwtBody
{
    ACompNwtBody() = default;
    ACompNwtBody(ACompNwtBody&& move);
    ACompNwtBody& operator=(ACompNwtBody&& move);

    NewtonBody *m_body{nullptr};
    ActiveEnt m_entity{entt::null};
    //ActiveScene &m_scene;

    DataPhyRigidBody m_bodyData;
    bool m_rigidbodyDirty{false};
};

using ACompRigidBody = ACompNwtBody;

//{
//    ACompRigidBody(ActiveEnt entity, ActiveScene &scene, NewtonBody* body) :
//            m_entity(entity),
//            m_scene(scene),
//            m_body(body),
//            m_netForce(),
//            m_netTorque() {}
//    ACompRigidBody(NwtUserData&& move) = delete;
//    ACompRigidBody& operator=(NwtUserData&& move) = delete;

//    ActiveEnt m_entity;
//    ActiveScene &m_scene;

//    NewtonBody *m_body;

//    Vector3 m_netForce;
//    Vector3 m_netTorque;
//};

//struct NwtUserDataWorld
//{
//    ActiveScene &m_scene;
//};

//struct CompNewtonBody
//{
//    NewtonBody *m_body{nullptr};
//    std::unique_ptr<NwtUserData> m_data;
//};

//using ACompRigidBody = std::unique_ptr<NwtUserData>;

//struct CompNewtonCollision
//{
//    NewtonCollision *shape{nullptr};
//};


struct ACompCollisionShape
{
    NewtonCollision* m_collision{nullptr};
    ECollisionShape m_shape{ECollisionShape::NONE};
};

class SysNewton
{

public:
    SysNewton(ActiveScene &scene);
    SysNewton(SysNewton const& copy) = delete;
    SysNewton(SysNewton&& move) = delete;

    ~SysNewton();

    /**
     * Scan children for CompColliders. combine it all into a single compound
     * collision
     * @param entity [in] Entity containing CompNewtonBody
     */
    void create_body(ActiveEnt entity);

    void update_world();

    /**
     * @param transform [out] A matrix to store the relative orienation of ent
                              with respect to the ancestor
     * @return Pair of {level-1 entity, pointer to body found}. If hierarchy
     *         error, then {entt:null, nullptr}. If no ACompRigidBody found,
     *         then {level-1 entity, nullptr}
     */
    std::pair<ActiveEnt, ACompRigidBody*> find_rigidbody_ancestor(
            ActiveEnt ent, Matrix4* transform = nullptr);

    /**
     * Helper function for a SysMachine to fill a rigid body member
     * 
     * If parentRigidBody has a rigid body, returns a pointer to it; if
     * rigidBody is null, attempts to find the ancestor of childEntity
     */
    std::pair<ACompRigidBody*, ACompTransform*> try_get_or_find_rigidbody_parent(
            ActiveEnt childEntity, ActiveEnt& parentRigidBody);

    Vector3 get_rigidbody_CoM(ACompRigidBody &body);

    /**
     * Update the inertia properties of a rigid body
     *
     * Given an existing rigid body, computes and updates the mass matrix and
     * center of mass. Entirely self contained, calls the other inertia
     * functions in this class.
     * @param entity [in] The rigid body to update
     */
    void update_rigidbody_inertia(ActiveEnt entity);

    constexpr ActiveScene& get_scene() { return m_scene; }

    void body_apply_force(ACompRigidBody &body, Vector3 force);
    void body_apply_force_local(ACompRigidBody &body, Vector3 force);

    void body_apply_accel(ACompRigidBody &body, Vector3 accel);
    void body_apply_accel_local(ACompRigidBody &body, Vector3 accel);

    void body_apply_torque(ACompRigidBody &body, Vector3 torque);
    void body_apply_torque_local(ACompRigidBody &body, Vector3 torque);

    void shape_create_box(ACompCollisionShape &shape, Vector3 size);
    void shape_create_sphere(ACompCollisionShape &shape, float radius);

    template<class TRIANGLE_IT_T>
    void shape_create_tri_mesh_static(ACompCollisionShape &shape,
                                      TRIANGLE_IT_T const& start,
                                      TRIANGLE_IT_T const& end);

private:

    /**
     * Search descendents for CompColliders and add NewtonCollisions to a vector.
     * @param ent [in] Entity to check colliders for, and recurse into children
     * @param compound [in] Compound collision
     * @param currentTransform [in] Hierarchy transform of ancestors
     */
    void find_and_add_colliders(ActiveEnt ent,
                                NewtonCollision *compound,
                                Matrix4 const &currentTransform);

    /**
     * Recursively compute the center of mass of a rigid body
     * @return A 4-vector containing xyz=CoM, w=mass
     */
    Magnum::Vector4 compute_rigidbody_CoM(ActiveEnt root, Matrix4 currentTransform);

    /**
     * Transform an inertia tensor
     *
     * Transforms an inertia tensor using the parallel axis theorem.
     * See "Tensor generalization" section on 
     * https://en.wikipedia.org/wiki/Parallel_axis_theorem for more information.
     * @param I [in] The original inertia tensor
     * @param mass [in] The total mass of the object
     * @param translation [in] The translation part of the transformation
     * @param rotation [in] The rotation part of the transformation
     * @return The transformed inertia tensor
     */
    Matrix3 transform_inertia_tensor(Matrix3 I, float mass, Vector3 translation,
        Matrix3 rotation);

    /**
     * Compute the volume of a part
     *
     * Traverses the immediate children of the specified entity and sums the
     * volumes of any detected collision volumes. Cannot account for overlapping
     * collider volumes.
     * @param part [in] The part
     * @return The part's collider volume
     */
    float compute_part_volume(ActiveEnt part);

    /**
     * Compute the center of mass of a single part (non-recursive)
     *
     * Scans the first level of the specified part's children and computes the
     * part's overall CoM from their ACompMass components
     * @param part [in] The part whose center of mass to compute
     * @return The part center of mass (3-vector)
     */
    Vector3 compute_part_CoM(ActiveEnt part);

    /**
     * Compute the inertia of a shape
     *
     * Given a mass and shape, returns the principal moments of inertia of a
     * volume by calling the appropriate inertia tensor calculation.
     * @param shape [in] The shape of the volume
     * @param scale [in] The size of the volume (blender scaling)
     * @param mass [in] The mass of the volume
     * @return The principal moments of inertia, {Ixx, Iyy, Izz}
     */
    Vector3 compute_col_shape_inertia(ECollisionShape shape, Vector3 scale, float mass);

    /**
     * Compute the moment of inertia of a part
     *
     * Searches the immediate children of the specified entity and computes
     * its moment of inertia. Assumes mass is evenly distributed over collision
     * volume. The part center of mass is precomputed and passed as a parameter
     * to maximize physical accuracy of the calculation.
     * @param part [in] The part
     * @return The inertia tensor of the part about its origin
     */
    Matrix3 compute_part_inertia(ActiveEnt part, Vector3 centerOfMass);

    /**
     * Compute the moment of inertia of a rigid body
     *
     * Searches the child nodes of the root and computes the total moment of
     * inertia of the body. Does not perform recursion, as vehicles do not yet
     * have nested hierarchies.
     * @param root [in] The root entity of the rigid body
     * @param centerOfMass [in] The center of mass of the rigid body
     * @return The inertia tensor of the rigid body about its center of mass
     */
    Matrix3 compute_rigidbody_inertia(ActiveEnt rbRoot, Vector3 centerOfMass,
        Matrix4 currentTransform);

    /**
     * Compute the inertia tensor for a cylinder
     *
     * Computes the moment of inertia about the principal axes of a cylinder
     * whose axis of symmetry lies along the z-axis
     * @return The moment of inertia about the 3 principal axes (x, y, z)
     */
    Vector3 cylinder_inertia_tensor(float radius, float height, float mass);

    void on_body_construct(entt::registry& reg, ActiveEnt ent);
    void on_body_destruct(entt::registry& reg, ActiveEnt ent);

    void on_shape_construct(entt::registry& reg, ActiveEnt ent);
    void on_shape_destruct(entt::registry& reg, ActiveEnt ent);

    NewtonCollision* newton_create_tree_collision(
            const NewtonWorld *newtonWorld, int shapeId);
    void newton_tree_collision_add_face(
            const NewtonCollision* treeCollision, int vertexCount,
            const float* vertexPtr, int strideInBytes, int faceAttribute);
    void newton_tree_collision_begin_build(
            const NewtonCollision* treeCollision);
    void newton_tree_collision_end_build(
            const NewtonCollision* treeCollision,  int optimize);


    ActiveScene& m_scene;
    NewtonWorld *const m_nwtWorld;

    UpdateOrderHandle m_updatePhysicsWorld;
};

template<class TRIANGLE_IT_T>
void SysNewton::shape_create_tri_mesh_static(ACompCollisionShape &shape,
                                             TRIANGLE_IT_T const& start,
                                             TRIANGLE_IT_T const& end)
{
    // TODO: this is actually horrendously slow and WILL cause issues later on.
    //       Tree collisions aren't made for real-time loading. Consider
    //       manually hacking up serialized data instead of add face, or use
    //       Newton's dgAABBPolygonSoup stuff directly

    NewtonCollision* tree = newton_create_tree_collision(m_nwtWorld, 0);

    newton_tree_collision_begin_build(tree);

    Vector3 triangle[3];

    for (TRIANGLE_IT_T it = start; it != end; it += 3)
    {
        triangle[0] = *reinterpret_cast<Vector3 const*>((it + 0).position());
        triangle[1] = *reinterpret_cast<Vector3 const*>((it + 1).position());
        triangle[2] = *reinterpret_cast<Vector3 const*>((it + 2).position());

        newton_tree_collision_add_face(tree, 3,
                                       reinterpret_cast<float*>(triangle),
                                       sizeof(float) * 3, 0);

    }

    newton_tree_collision_end_build(tree, 2);

    shape.m_shape = ECollisionShape::TERRAIN;
    shape.m_collision = tree;
}

}

