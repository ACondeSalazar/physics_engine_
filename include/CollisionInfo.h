#ifndef COLLISION_INFO_H
#define COLLISION_INFO_H
#include "Components.h"

struct CollisionInfo
{
        int index_box1;
        int index_box2;
        Vector2 normal; //the normal vector between the center of the 2 objects
        float relative_normal_velocity; //relative velocity along the collision normal hence why its a scalar
        float relative_angular_velocity;
        float penetration_depth;
        float restitution;
        float linear_impulse;
        std::vector<Vector2> collision_points;
        bool colliding;


        CollisionInfo()
        {
            normal = Vector2();
            relative_normal_velocity = 0;
            penetration_depth = 10000;
            restitution = 0;
            linear_impulse = 0;
            colliding = true;
        }

        //normal
        Vector2 get_normal(){return normal;}
        void set_normal(Vector2 a_normal)
        {
            normal = a_normal;
        }

        //penetration_depth
        float get_penetration_depth(){return penetration_depth;}
        void set_penetration_depth(float a_pen_depth)
        {
            penetration_depth = a_pen_depth;
        }

        //restitution
        float get_restitution(){return restitution;}
        void set_restitution(float a_restitution)
        {
            restitution = a_restitution;
        }

        //linear_impulse
        float get_linear_impulse(){return linear_impulse;}
        void set_linear_impulse(float a_linear_impulse)
        {
            linear_impulse = a_linear_impulse;
        }


};

#endif // COLLISION_INFO_H
