#ifndef ENGINE_H
#define ENGINE_H
#include "Components.h"
#include "CollisionInfo.h"
#include <vector>
using namespace std;
struct Physics_simulation_engine
{
    private:
        std::vector<Box> objects; //les objets
        std::vector<CollisionInfo> collisions_vector; //liste des info de collision entre les objets si il y a
        std::vector<Vertices> vertices_vector; // liste des vertices de tout les objets
        std::vector<Edges> edges_vector; //liste des cotes de tous les objets
        double delta;
        Vector2 simulation_size;
        float velocity_treshold = 8;
        float gravity_force = 500;

    public:
        Physics_simulation_engine(float width, float height)
        {
            simulation_size = Vector2(width, height);
        }

        void update(double delta)
        {
            vertices_vector.clear();
            edges_vector.clear();
            for(int i = 0; i < objects.size(); i++)
            {
                vertices_vector.push_back(objects[i].get_vertices());
                edges_vector.push_back( Edges(vertices_vector[i]) );
            }

            //updating positions based on velocity
            for(Box& box :objects)
            {
                if(box.is_immovable())
                {
                    box.set_velocity(0,0);
                    continue;
                }
                Vector2 posi = box.get_position();
                //if (posi.x < -300 || posi.x > simulation_size.x +300) {continue;} //if out of bound we dont update the object
                //if (posi.y > simulation_size.y + 300) {continue;}

                Vector2 velocity = box.get_velocity();
                float angular_velocity = box.get_angular_velocity();

                if(velocity.get_norm() > velocity_treshold)
                {
                    box.add_to_position(velocity*delta);
                }
                if(velocity.y < gravity_force)
                {
                    box.add_to_velocity(Vector2(0,gravity_force)*delta *box.get_inverse_mass());
                }


                box.add_to_rotation(angular_velocity*delta);
            }

            //detecting collision

            int maxloop = 1;
            for(int loop = 0; loop < maxloop; loop++) //correct pos multiple time for accuracy
            {

                collisions_vector.clear();
                for(int i = 0; i < objects.size(); i++)
                {
                    for(int j = i+1; j < objects.size(); j++)
                    {
                        collisions_vector.push_back(detect_collision(i,j));
                    }
                }

                //cout << objects.size() << endl;
                for(CollisionInfo& info : collisions_vector){
                    if(info.colliding)
                    {
                        resolve_collision(info);
                        correct_position(info, 1);
                    }
                }

            }

        }

        CollisionInfo detect_collision(int index_box_1, int index_box_2)
        {
            CollisionInfo collision_info = CollisionInfo();
            collision_info.index_box1 = index_box_1; //on enregistre les acteurs de la collision
            collision_info.index_box2 = index_box_2;

            //on recupere l'objet, ses vertices et ses edges
            Box box1 = objects[index_box_1];
            Vertices vertices_box1 = vertices_vector[index_box_1];
            Edges edges_box1 = edges_vector[index_box_1];
            Vector2 position1 = box1.get_position();
            float restitution1 = box1.get_restitution();


            //pareil pour le deuxieme
            Box box2 = objects[index_box_2];
            Vertices vertices_box2 = vertices_vector[index_box_2];
            Edges edges_box2 = edges_vector[index_box_2];
            Vector2 position2 = box2.get_position();
            float restitution2 = box2.get_restitution();

            //liste des axes
            std::vector<Vector2> axes; //tous les axes ou on va appliquer le sat theorem

            //on recupere les axes des 2 boites
            for(Line line : edges_box1.get_edges())
            {
                Vector2 axis = line.get_vector()/line.get_vector().get_norm();//line.normalized_normal();
                axes.push_back(axis);
            }

            for(Line line : edges_box2.get_edges())
            {
                Vector2 axis = line.get_vector()/line.get_vector().get_norm();//line.normalized_normal();
                axes.push_back(axis);
            }

            for(Vector2 axis : axes) //on projette sur chaque axe
            {
                vector<float> projection_box1 = project_box_on_axis(index_box_1,axis); //tableau de taille 2 [min,max] du segment projete
                vector<float> projection_box2 = project_box_on_axis(index_box_2,axis); //pareil

                if (projection_box1[1] < projection_box2[0] || projection_box2[1] < projection_box1[0]) //if no collision
                {
                    collision_info.colliding = false;
                    //std::cout << "not colliding! "<< std::endl;
                    return collision_info;
                }

                float projection_min = std::min(projection_box1[1], projection_box2[1]);
                float projection_max = std::max(projection_box1[0], projection_box2[0]);
                float penetration_depth = projection_min - projection_max;
                //std::cout << "penetration depth : "<< penetration_depth<< std::endl;

                if(penetration_depth < collision_info.penetration_depth)
                {
                    collision_info.penetration_depth = penetration_depth;
                    //std::cout << "NEW DEPTH"<< std::endl;
                    collision_info.set_normal(axis);

                    collision_info.collision_points.push_back(position1 + (axis*projection_min) );
                    collision_info.collision_points.push_back(position2 + (axis*projection_min) );
                }

            }



            if(collision_info.normal.dot_product(position2-position1) < 0) //on s'assure de toujours pointe de l'objet 1 vers l'objet 2 pour eviter le bordel apres
            {
                collision_info.normal = -collision_info.normal;
            }

            collision_info.relative_normal_velocity = collision_info.normal.dot_product(box2.get_velocity() - box1.get_velocity()); //relative velocity along collision normal
            collision_info.relative_angular_velocity = box2.get_angular_velocity() - box1.get_angular_velocity();

            if(collision_info.relative_normal_velocity > 0) // si les objets s'eloignent la collision a deja ete resolu
            {
                collision_info.colliding = false;
                return collision_info;
            }

            collision_info.restitution = std::min(restitution1,restitution2); //ceofficient de restitution

            return collision_info;
        }

        //permet de projeter tous les cotes d une box sur un axe (attention ne marche que avec les polygones convexes)
        vector<float> project_box_on_axis(int box_index, Vector2 axis)//projete tous les cotes d'une box sur un axe
        {
            //-----
            std::vector<Line> edges = edges_vector[box_index].get_edges();
            vector<float> projection = project_line_on_axis(edges[0], axis);
            float projection_min = projection[0];
            float projection_max = projection[1];

            for(int i = 1; i < edges.size(); i++)
            {
                projection = project_line_on_axis(edges[i],axis); //on projete
                projection_min = std::min(projection_min, projection[0]); //on regarde si on a un nouveau minimum
                projection_max = std::max(projection_max, projection[1]); //pareil pour le max
            }

            return vector<float>{projection_min, projection_max};

        }

        //permet de projeter un segment (ici une line) sur un axe
        vector<float> project_line_on_axis(Line line, Vector2 axis)
        {
            float projection_start = line.start_point.dot_product(axis); //on projete le premier point
            float projection_end = line.end_point.dot_product(axis); //le deuxieme

            float projection_min = std::min(projection_start,projection_end);
            float projection_max = std::max(projection_start,projection_end);

            return vector<float>{projection_min, projection_max};
        }

        void resolve_collision(CollisionInfo collision_info)
        {
            Box& box1 = objects[collision_info.index_box1];
            Box& box2 = objects[collision_info.index_box2];

            Vector2 shape1 = box1.get_size_sides();
            Vector2 shape2 = box2.get_size_sides();

            Vector2 collision_normal = collision_info.normal;

            float mass1 = box1.get_mass();
            float mass2 = box2.get_mass();

            float linear_impulse = -(1+collision_info.restitution)*collision_info.relative_normal_velocity;
            linear_impulse /= box1.get_inverse_mass() + box2.get_inverse_mass();

            //adding velocity

            Vector2 impulse_vec =  collision_normal * linear_impulse; //vector with direction and magnitude of force resulted by the collision
            if(!box1.is_immovable()){
                box1.add_to_velocity(-impulse_vec/mass1);
                //box1.add_to_position(collision_normal * (-position_correction));
            }
            if(!box2.is_immovable()){
                box2.add_to_velocity(impulse_vec/mass2);
                //box2.add_to_position(collision_normal * position_correction);
            }

        }

        void correct_position(CollisionInfo collision_info, float strength)
        {
            Box& box1 = objects[collision_info.index_box1];
            Box& box2 = objects[collision_info.index_box2];

            float correction_percentage = 0.1; //from 0 to 1 how much we want to correct position too high value may look weird
            float correction_treshold = 0.1; //let objects sink into each other with this threshold to avoid jitter

            if(collision_info.penetration_depth < correction_treshold){return;} //si on doit faire qu'une tres faible correction on skip

            float position_correction = collision_info.penetration_depth - correction_treshold / (box1.get_inverse_mass() + box2.get_inverse_mass() ) * correction_percentage * strength;

            if(box1.is_immovable()){
                box2.add_to_position(collision_info.normal * (position_correction));
            }
            else if(box2.is_immovable()){
                box1.add_to_position(collision_info.normal * (-position_correction));
            }else{
                box1.add_to_position(collision_info.normal * (-position_correction) *0.5);
                box2.add_to_position(collision_info.normal * (position_correction) *0.5);
            }
        }

        void add_object(Box a_box)
        {
            objects.push_back(a_box);
        }

        std::vector<Box> get_objects()
        {
            return objects;
        }

        std::vector<Edges> get_edges_vector()
        {
            return edges_vector;
        }

        std::vector<Vertices> get_vertices_vector()
        {
            return vertices_vector;
        }

        std::vector<CollisionInfo> get_collisions_vector()
        {
            return collisions_vector;
        }

        void delete_all_objects()
        {
            objects.clear();
        }

};


#endif // ENGINE_H
