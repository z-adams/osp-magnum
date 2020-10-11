#include "SysNewton.h"

#include "ActiveScene.h"

#include <Newton.h>

using namespace osp;
using namespace osp::active;
using Magnum::Vector4;

void cb_force_torque(const NewtonBody* body, dFloat timestep, int threadIndex)
{
    SysNewton* sysNewton = static_cast<SysNewton*>(
                NewtonWorldGetUserData(NewtonBodyGetWorld(body)));
    ActiveScene &scene = sysNewton->get_scene();

    ACompNwtBody *bodyComp
            = static_cast<ACompNwtBody*>(NewtonBodyGetUserData(body));
    DataPhyRigidBody &bodyPhy = bodyComp->m_bodyData;

    //Matrix4 matrix;

    ACompTransform& transform = scene.get_registry()
                                        .get<ACompTransform>(bodyComp->m_entity);
    NewtonBodyGetMatrix(body, transform.m_transform.data());

    Matrix4 matrix;
    NewtonBodyGetMatrix(body, matrix.data());

    // Check if floating origin translation is in progress
    if (scene.floating_origin_in_progress())
    {
        // Do floating origin translation, then set position

        matrix[3].xyz() += scene.floating_origin_get_total();

        NewtonBodySetMatrix(body, matrix.data());

        //std::cout << "fish\n";
    }

    // Apply force and torque
    NewtonBodySetForce(body, bodyPhy.m_netForce.data());
    NewtonBodySetTorque(body, bodyPhy.m_netTorque.data());

    // Reset accumolated net force and torque for next frame
    bodyPhy.m_netForce = {0.0f, 0.0f, 0.0f};
    bodyPhy.m_netTorque = {0.0f, 0.0f, 0.0f};

    // temporary fun stuff:
    //NewtonBodySetForce(body, (-(matrix[3].xyz() + Vector3(0, 0, 20)) * 0.1f).data());
    //Vector3 torque(0, 1, 0);
    //NewtonBodySetTorque(body, torque.data());

}

ACompNwtBody::ACompNwtBody(ACompNwtBody&& move) :
        m_body(move.m_body),
        m_entity(move.m_entity),
        //m_scene(move.m_scene),
        m_bodyData(move.m_bodyData)
{
    if (m_body)
    {
        NewtonBodySetUserData(m_body, this);
    }
}

ACompNwtBody& ACompNwtBody::operator=(ACompNwtBody&& move)
{
    m_body = move.m_body;
    m_entity = move.m_entity;
    m_bodyData = move.m_bodyData;

    if (m_body)
    {
        NewtonBodySetUserData(m_body, this);
    }

    return *this;
}

SysNewton::SysNewton(ActiveScene &scene) :
        m_scene(scene),
        m_nwtWorld(NewtonCreate()),
        m_updatePhysicsWorld(scene.get_update_order(), "physics", "wire", "",
                             std::bind(&SysNewton::update_world, this))
{
    //std::cout << "sysnewtoninit\n";
    NewtonWorldSetUserData(m_nwtWorld, this);

    scene.get_registry().on_construct<ACompRigidBody>()
                    .connect<&SysNewton::on_body_construct>(*this);
    scene.get_registry().on_destroy<ACompRigidBody>()
                    .connect<&SysNewton::on_body_destruct>(*this);

    scene.get_registry().on_construct<ACompCollisionShape>()
                    .connect<&SysNewton::on_shape_construct>(*this);
    scene.get_registry().on_destroy<ACompCollisionShape>()
                    .connect<&SysNewton::on_shape_destruct>(*this);
}

SysNewton::~SysNewton()
{
    //std::cout << "sysnewtondestroy\n";
    // Clean up newton dynamics stuff
    m_scene.get_registry().clear<ACompRigidBody>();
    m_scene.get_registry().clear<ACompCollisionShape>();
    NewtonDestroyAllBodies(m_nwtWorld);
    NewtonDestroy(m_nwtWorld);
}

void SysNewton::find_and_add_colliders(ActiveEnt ent, NewtonCollision *compound, Matrix4 const &currentTransform)
{

    ActiveEnt nextChild = ent;

    while(nextChild != entt::null)
    {
        ACompHierarchy const &childHeir = m_scene.reg_get<ACompHierarchy>(nextChild);
        ACompTransform const &childTransform = m_scene.reg_get<ACompTransform>(nextChild);

        ACompCollisionShape* childCollide = m_scene.get_registry().try_get<ACompCollisionShape>(nextChild);
        Matrix4 childMatrix = currentTransform * childTransform.m_transform;

        if (childCollide)
        {
            NewtonCollision *collision = childCollide->m_collision;

            if (collision)
            {

            }
            else
            {
                // TODO: care about collision shape
                childCollide->m_collision = collision = NewtonCreateSphere(m_nwtWorld, 0.5f, 0, NULL);
            }

            //Matrix4 nextTransformNorm = nextTransform.

            // Set transform relative to root body
            Matrix4 f = Matrix4::translation(childMatrix.translation());
            NewtonCollisionSetMatrix(collision, f.data());

            // Add body to compound collision
            NewtonCompoundCollisionAddSubCollision(compound, collision);

        }
        else
        {
            //return;
        }

        find_and_add_colliders(childHeir.m_childFirst, compound, childMatrix);
        nextChild = childHeir.m_siblingNext;
    }
}

Vector4 SysNewton::compute_rigidbody_CoM(ActiveEnt root, Matrix4 currentTransform)
{
    ActiveEnt nextChild = root;
    Vector3 localCoM{0.0f};
    float localMass = 0.0f;

    while (nextChild != entt::null)
    {
        ACompHierarchy const &childHeir = m_scene.reg_get<ACompHierarchy>(nextChild);
        ACompTransform const &childTransform = m_scene.reg_get<ACompTransform>(nextChild);

        Matrix4 childMatrix = currentTransform * childTransform.m_transform;
        ACompMass* massComp = m_scene.get_registry().try_get<ACompMass>(nextChild);

        if (massComp)
        {
            float childMass = m_scene.reg_get<ACompMass>(nextChild).m_mass;

            Vector3 offset = childMatrix.translation();

            localCoM += childMass * offset;
            localMass += childMass;
        }

        Vector4 subCoM = compute_rigidbody_CoM(childHeir.m_childFirst, childMatrix);
        localCoM += subCoM.w() * subCoM.xyz();
        localMass += subCoM.w();

        nextChild = childHeir.m_siblingNext;
    }
    if (localMass > 0.0f)
    {
        return {localCoM / localMass, localMass};
    }
    return Vector4{0.0f};
}

Matrix3 SysNewton::transform_inertia_tensor(Matrix3 I, float mass, Vector3 translation,
    Matrix3 rotation)
{
    // Apply rotation via similarity transformation
    I = rotation.transposed() * I * rotation;

    // Translate via tensor generalized parallel axis theorem
    using Magnum::Math::dot;
    Vector3 r = translation;
    Matrix3 outerProdR = {r*r.x(), r*r.y(), r*r.z()};
    Matrix3 E3 = Matrix3{};

    return I + mass*(dot(r, r)*E3 - outerProdR);
}

float SysNewton::compute_part_volume(ActiveEnt part)
{
    float volume = 0.0f;
    auto checkVol = [&](ActiveEnt ent)
    {
        ACompCollisionShape* shape = m_scene.get_registry().try_get<ACompCollisionShape>(ent);
        if (shape)
        {
            ACompTransform const& xform = m_scene.reg_get<ACompTransform>(ent);
            volume += col_shape_volume(shape->m_shape, xform.m_transform.scaling());
        }
        return true;
    };

    m_scene.hierarchy_traverse(part, checkVol, false);

    return volume;
}

// TODO: this iteration scheme is begging for generalization
Vector3 SysNewton::compute_part_CoM(ActiveEnt part)
{
    ActiveEnt nextChild = part;
    Vector3 centerOfMass{0.0f};
    float totalMass = 0.0f;

    while (nextChild != entt::null)
    {
        ACompHierarchy const &childHeir = m_scene.reg_get<ACompHierarchy>(nextChild);
        ACompTransform const &childTransform = m_scene.reg_get<ACompTransform>(nextChild);

        Matrix4 childMatrix = childTransform.m_transform;
        ACompMass* massComp = m_scene.get_registry().try_get<ACompMass>(nextChild);

        if (massComp)
        {
            float childMass = m_scene.reg_get<ACompMass>(nextChild).m_mass;

            Vector3 offset = childMatrix.translation();

            centerOfMass += childMass * offset;
            totalMass += childMass;
        }

        nextChild = childHeir.m_siblingNext;
    }
    if (totalMass > 0.0f)
    {
        return {centerOfMass / totalMass};
    }
    return Vector3{0.0f};
}

Vector3 SysNewton::compute_col_shape_inertia(ECollisionShape shape, Vector3 scale, float mass)
{
    switch (shape)
    {
    case ECollisionShape::CYLINDER:
    {
        // Default cylinder: radius 1, height 2
        float height = 2.0f * scale.z();
        // if sclY != sclX I will be mad
        float radius = scale.x();
        return cylinder_inertia_tensor(radius, height, mass);
    }
    }
    return Vector3{0.0f};
}

Matrix3 SysNewton::compute_part_inertia(ActiveEnt part, Vector3 centerOfMass)
{
    ACompMass const* partMass = m_scene.reg_try_get<ACompMass>(part);
    if (!partMass)
    {
        std::cout << "ERROR: not a part, can't compute inertia\n";
        return Matrix3{0.0f};
    }

    float partVolume = compute_part_volume(part);
    if (partVolume == 0.0f)
    {
        return Matrix3{0.0f};
    }
    float partDensity = partMass->m_mass / partVolume;

    Matrix3 I{0.0f};
    ActiveEnt nextChild = m_scene.reg_get<ACompHierarchy>(part).m_childFirst;

    while (nextChild != entt::null)
    {
        ACompHierarchy const& childHier = m_scene.reg_get<ACompHierarchy>(nextChild);
        ACompTransform const& childTransform = m_scene.reg_get<ACompTransform>(nextChild);

        Matrix4 childMatrix = childTransform.m_transform;
        ACompCollisionShape* childCollide = m_scene.reg_try_get<ACompCollisionShape>(nextChild);
        ACompContainerShape* childContainer = m_scene.reg_try_get<ACompContainerShape>(nextChild);

        if (childCollide || childContainer)
        {
            float mass = 0.0f;
            Vector3 principalAxes{0.0f};

            if (childCollide)
            {
                float volume = col_shape_volume(childCollide->m_shape, childMatrix.scaling());
                mass = volume * partDensity;

                principalAxes = compute_col_shape_inertia(
                    childCollide->m_shape, childMatrix.scaling(), mass);
            }
            else if (childContainer)
            {
                ACompMass contMass = m_scene.reg_get<ACompMass>(nextChild);
                mass = contMass.m_mass;
                
                principalAxes = compute_col_shape_inertia(
                    childContainer->m_shape, childMatrix.scaling(), mass);
            }

            Matrix3 inertiaTensor{};
            inertiaTensor[0][0] = principalAxes.x();
            inertiaTensor[1][1] = principalAxes.y();
            inertiaTensor[2][2] = principalAxes.z();

            Matrix3 rotation = childMatrix.rotation();
            Vector3 offset = childMatrix.translation() - centerOfMass;
            I += transform_inertia_tensor(inertiaTensor, mass, offset, rotation);
        }

        nextChild = childHier.m_siblingNext;
    }

    return I;
}

/*
 NOTE: The problem here is that the inertia tensor calculation depends on mass
       distribution, but our point-mass-plus-colliders model doesn't really allow
       for the specification of mass distribution. For now, the solution is for
       compute_part_inertia() to find the volume of the part's colliders and
       compute the average density of the part, which is then used to assume a
       uniform mass distribution. Future solutions may involve requiring
       colliders to specify density.

 TODO: This function is not recursive yet, because rigid bodies are currently
       only one level deep. If that changes, this function will need to be 
       updated accordingly.
*/
Matrix3 SysNewton::compute_rigidbody_inertia(ActiveEnt rbRoot, Vector3 centerOfMass,
    Matrix4 currentTransform)
{
    ActiveEnt nextChild = rbRoot;
    Matrix3 localI{0.0f};
    float localMass = 0.0f;

    while (nextChild != entt::null)
    {
        ACompHierarchy const &childHier = m_scene.reg_get<ACompHierarchy>(nextChild);
        ACompTransform const &childTransform = m_scene.reg_get<ACompTransform>(nextChild);

        Matrix4 childMatrix = currentTransform * childTransform.m_transform;

        /* At present, all first-level children of a vehicle are parts, so
           as long as an ACompMass is present, we know nextChild is a part
           and should calculate its inertia
         */
        ACompMass* massComp = m_scene.reg_try_get<ACompMass>(nextChild);
        if (massComp)
        {
            Vector3 childCoM = compute_part_CoM(nextChild);
            Vector3 offset = (childMatrix.translation() + childCoM) - centerOfMass;
            Matrix3 rotation = childMatrix.rotation();

            Matrix3 partI = compute_part_inertia(nextChild, childCoM);
            float partMass = m_scene.reg_get<ACompMass>(nextChild).m_mass;
            localI += transform_inertia_tensor(partI, partMass, offset, rotation);
        }

        nextChild = childHier.m_siblingNext;
    }
    return localI;
}

Vector3 SysNewton::cylinder_inertia_tensor(float radius, float height, float mass)
{
    float r = radius, h = height;
    float xx = (1.0f/12.0f) * (3.0f*r*r + h*h);
    float yy = xx;
    float zz = r*r / 2.0f;

    return mass * Vector3{xx, yy, zz};
}

void SysNewton::update_rigidbody_inertia(ActiveEnt entity)
{
    ACompHierarchy& hier = m_scene.reg_get<ACompHierarchy>(entity);
    ACompNwtBody& body = m_scene.reg_get<ACompNwtBody>(entity);

    Vector4 centerOfMass = compute_rigidbody_CoM(hier.m_childFirst, Matrix4());
    Matrix3 inertia = compute_rigidbody_inertia(hier.m_childFirst, centerOfMass.xyz(),
        Matrix4());
    float Ixx = inertia[0][0];
    float Iyy = inertia[1][1];
    float Izz = inertia[2][2];
    body.m_bodyData.m_mass = centerOfMass.w();
    NewtonBodySetMassMatrix(body.m_body, body.m_bodyData.m_mass, Ixx, Iyy, Izz);
    NewtonBodySetCentreOfMass(body.m_body, centerOfMass.xyz().data());
}

void SysNewton::create_body(ActiveEnt entity)
{

    ACompHierarchy& entHier = m_scene.reg_get<ACompHierarchy>(entity);
    ACompNwtBody& entBody = m_scene.reg_get<ACompNwtBody>(entity);
    ACompCollisionShape* entShape = m_scene.get_registry().try_get<ACompCollisionShape>(entity);
    ACompTransform& entTransform = m_scene.reg_get<ACompTransform>(entity);

    if (!entShape)
    {
        // need collision shape to make a body
        return;
    }

    //if (entBody.m_body)
    //{
        // body is already initialized, delete it first and make a new one
    //}


    switch (entShape->m_shape)
    {
    case ECollisionShape::COMBINED:
    {
        NewtonCollision* compound = NewtonCreateCompoundCollision(m_nwtWorld, 0);

        NewtonCompoundCollisionBeginAddRemove(compound);
        find_and_add_colliders(entHier.m_childFirst, compound, Matrix4());
        NewtonCompoundCollisionEndAddRemove(compound);

        if (entBody.m_body)
        {
            NewtonBodySetCollision(entBody.m_body, compound);
        }
        else
        {
            entBody.m_body = NewtonCreateDynamicBody(m_nwtWorld, compound,
                                                       Matrix4().data());
        }

        NewtonDestroyCollision(compound);
        update_rigidbody_inertia(entity);
        break;
    }
    case ECollisionShape::TERRAIN:
    {
        if (entShape->m_collision)
        {
            if (entBody.m_body)
            {
                NewtonBodySetCollision(entBody.m_body, entShape->m_collision);
            }
            else
            {
                entBody.m_body = NewtonCreateDynamicBody(m_nwtWorld,
                        entShape->m_collision,
                        Matrix4().data());
            }


        }
        else
        {
            // make a collision shape somehow
        }
    }
    default:
        break;
    }

    entBody.m_entity = entity;

    // Set position/rotation
    NewtonBodySetMatrix(entBody.m_body, entTransform.m_transform.data());

    // Set damping to 0, as default is 0.1
    // reference frame may be moving and air pressure stuff
    NewtonBodySetLinearDamping(entBody.m_body, 0.0f);

    // Make it easier to rotate
    NewtonBodySetAngularDamping(entBody.m_body, Vector3(1.0f, 1.0f, 1.0f).data());

    // Set callback for updating position of entity and everything else
    NewtonBodySetForceAndTorqueCallback(entBody.m_body, cb_force_torque);

    // Set user data
    //NwtUserData *data = new NwtUserData(entity, m_scene);
    NewtonBodySetUserData(entBody.m_body, &entBody);

    // don't leak memory
    //NewtonDestroyCollision(ball);
}

void SysNewton::update_world()
{
    auto view = m_scene.get_registry().view<ACompNwtBody>();
    for (ActiveEnt ent : view)
    {
        ACompNwtBody& body = view.get<ACompNwtBody>(ent);
        if (body.m_rigidbodyDirty)
        {
            update_rigidbody_inertia(ent);
            body.m_rigidbodyDirty = false;
            std::cout << "Updating RB: new CoM Z: " << get_rigidbody_CoM(body).z() << "\n";
        }
    }

    m_scene.floating_origin_translate_begin();

    NewtonUpdate(m_nwtWorld, m_scene.get_time_delta_fixed());
    //std::cout << "hi\n";
}

std::pair<ActiveEnt, ACompRigidBody*> SysNewton::find_rigidbody_ancestor(
        ActiveEnt ent, Matrix4* transform)
{
    ActiveEnt prevEnt, currEnt = ent;
    ACompHierarchy *currHier = nullptr;

    do
    {
       currHier = m_scene.get_registry().try_get<ACompHierarchy>(currEnt);

        if (!currHier)
        {
            return {entt::null, nullptr};
        }
        if (transform && currHier->m_level > gc_heir_physics_level)
        {
            Matrix4 localTransform = m_scene.reg_get<ACompTransform>(currEnt).m_transform;
            *transform = localTransform * (*transform);
        }

        prevEnt = currEnt;
        currEnt = currHier->m_parent;
    }
    while (currHier->m_level != gc_heir_physics_level);

    ACompRigidBody *body = m_scene.get_registry().try_get<ACompRigidBody>(prevEnt);

    return {prevEnt, body};

}

std::pair<ACompRigidBody*, ACompTransform*> SysNewton::try_get_or_find_rigidbody_parent(
    ActiveEnt childEntity, ActiveEnt& parentRigidBody)
{
    ACompRigidBody *compRb;
    ACompTransform *compTf;

    if (m_scene.get_registry().valid(parentRigidBody))
    {
        // Try to get the ACompRigidBody if valid
        compRb = m_scene.get_registry()
            .try_get<ACompRigidBody>(parentRigidBody);
        compTf = m_scene.get_registry()
            .try_get<ACompTransform>(parentRigidBody);
        if (!compRb || !compTf)
        {
            parentRigidBody = entt::null;
            return {nullptr, nullptr};
        }
    }
    else
    {
        // rocket's rigid body not set yet
        auto body = find_rigidbody_ancestor(childEntity);

        if (body.second == nullptr)
        {
            std::cout << "no rigid body!\n";
            return {nullptr, nullptr};
        }

        parentRigidBody = body.first;
        compRb = body.second;
        compTf = m_scene.get_registry()
            .try_get<ACompTransform>(body.first);
    }
    return {compRb, compTf};
}

Vector3 osp::active::SysNewton::get_rigidbody_CoM(ACompRigidBody &body)
{
    Vector3 com;
    NewtonBodyGetCentreOfMass(body.m_body, com.data());
    return com;
}

// TODO

void SysNewton::body_apply_force(ACompRigidBody &body, Vector3 force)
{
    body.m_bodyData.m_netForce += force;
}

void SysNewton::body_apply_force_local(ACompRigidBody &body, Vector3 force)
{

}

void SysNewton::body_apply_accel(ACompRigidBody &body, Vector3 accel)
{
    body_apply_force(body, accel * body.m_bodyData.m_mass);
}

void SysNewton::body_apply_accel_local(ACompRigidBody &body, Vector3 accel)
{

}

void SysNewton::body_apply_torque(ACompRigidBody &body, Vector3 torque)
{
    body.m_bodyData.m_netTorque += torque;
}

void SysNewton::body_apply_torque_local(ACompRigidBody &body, Vector3 torque)
{

}



void SysNewton::on_body_construct(entt::registry& reg, ActiveEnt ent)
{
    // TODO
    reg.get<ACompRigidBody>(ent).m_bodyData.m_mass = 1.0f;
}

void SysNewton::on_body_destruct(entt::registry& reg, ActiveEnt ent)
{
    // make sure the newton body is destroyed
    NewtonBody *body = reg.get<ACompRigidBody>(ent).m_body;
    if (body)
    {
        NewtonDestroyBody(body);
    }
}

void SysNewton::on_shape_construct(entt::registry& reg, ActiveEnt ent)
{

}

void SysNewton::on_shape_destruct(entt::registry& reg, ActiveEnt ent)
{
    // make sure the collision shape destroyed
    NewtonCollision *shape = reg.get<ACompCollisionShape>(ent).m_collision;
    if (shape)
    {
        NewtonDestroyCollision(shape);
    }
}

NewtonCollision* SysNewton::newton_create_tree_collision(
        const NewtonWorld *newtonWorld, int shapeId)
{
    return NewtonCreateTreeCollision(newtonWorld, shapeId);
}

void foo(void* const serializeHandle, const void * const buffer, int size)
{
    std::cout << std::hex;
    for (size_t i = 0; i < size; i ++)
    {
        int f = uint8_t(*((uint8_t*)buffer + i));
         std::cout << f;
    }
    std::cout << std::dec;

}


void SysNewton::newton_tree_collision_add_face(
        const NewtonCollision* treeCollision, int vertexCount,
        const float* vertexPtr, int strideInBytes, int faceAttribute)
{
    NewtonTreeCollisionAddFace(treeCollision, vertexCount, vertexPtr, strideInBytes, faceAttribute);
}

void SysNewton::newton_tree_collision_begin_build(
        const NewtonCollision* treeCollision)
{
    NewtonTreeCollisionBeginBuild(treeCollision);
}

void SysNewton::newton_tree_collision_end_build(
        const NewtonCollision* treeCollision, int optimize)
{
    NewtonTreeCollisionEndBuild(treeCollision, optimize);
}

