#ifndef ENGINE_H
#define ENGINE_H
#include "Components.hpp"
#include <chrono>
#include <codecvt>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <ostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

class Physics_simulation_engine {
private:
  std::vector<BoxBody> objects;                   // liste des objets présents
  std::vector<CollisionInfo> collision_info_list; // listg des CollisionInfo
  std::vector<Vertices> vertices_list; // liste des sommets de tout mes objets
  std::vector<Edges> edges_list;       // liste des cotés des objets
  double delta;
  Vector2f simulation_size;
  int id_count = 1;
  // le nombre de fois a subdivider delta
public:
  int accuracy = 5;
  // si la vélocité est négligeable on ne l'applique pas
  float apply_velocity_treshold = 8;
  float apply_angular_velocity_treshold = 15;
  // from 0 to 1 how much we want to correct position too high value may look

  // weird not used currently
  float correction_percentage = 1.5;
  // let objects sink into each other with this threshold to avoid jitter
  float correction_treshold = 0.1;

  // how many times we detect and resolve collision in a single step
  int correction = 1;

  Vector2f gravity_force = Vector2f(0, 500);

  // monitoring
  int updating_position_duration;
  int detect_collision_duration;
  int resolve_collision_duration;
  int correct_position_duration;

  Physics_simulation_engine(float width, float height) {
    simulation_size = Vector2f(width, height);
  }

  void update(double delta) {
    float delta_subdivision = delta / accuracy;
    for (int i = 0; i < accuracy; i++) {
      step(delta_subdivision);
    }
  }
  //#step
  void step(float delta) {
    // on recalcul les sommets et cotés des objets
    vertices_list.clear();
    edges_list.clear();
    for (int i = 0; i < objects.size(); i++) {
      vertices_list.push_back(objects[i].get_vertices());
      edges_list.push_back(Edges(vertices_list[i]));
    }
    // std::cout << "\x1B[2J\x1B[H" << std::endl; // clear console
    //   mise a jour des positions
    for (BoxBody &box : objects) {
      Vector2f position = box.get_position();
      Vector2f velocity = box.get_velocity();
      float angular_velocity = box.get_angular_velocity();
      Vector2f shape = box.get_size_sides();

      // std::cout << "vitesse " << velocity.get_norm() << std::endl;
      //  applying gravity
      box.add_to_velocity((gravity_force)*delta);

      if (box.is_immovable()) {
        box.set_velocity(0, 0);
        box.set_angular_velocity(0);
        continue;
      }

      if (velocity.get_norm() > apply_velocity_treshold) {
        box.add_to_position(velocity * delta);
      }
      if (std::abs(angular_velocity) > apply_angular_velocity_treshold) {
        box.add_to_rotation(box.get_angular_velocity() * delta);
      }

      int real_sim_border =
          150; // décalage de la bordure de la sim avec la taille de la fenetre
               // (la sim est plus grand que la fenetre)
      if (position.x > simulation_size.x + real_sim_border) {
        continue;
        // box.set_position(Vector2f(-real_sim_border, box.get_position().y));
      }
      if (position.x < -real_sim_border) {
        continue;
        // box.set_position(Vector2f(simulation_size.x+real_sim_border,
        // box.get_position().y));
      }
      if (position.y > simulation_size.y + real_sim_border) {
        continue;
        // box.set_position(Vector2f(box.get_position().x, -real_sim_border));
      }
      if (position.y < -real_sim_border) {
        continue;
        // box.set_position(Vector2f(box.get_position().x,
        // simulation_size.y+real_sim_border));
      }
    }

    int vrai_collision = 0;
    for (int loop = 0; loop < correction; loop++) {
      collision_info_list.clear();
      for (int i = 0; i < objects.size(); i++) {
        for (int j = i + 1; j < objects.size(); j++) {
          CollisionInfo collision_info = detect_collision(i, j);
          collision_info_list.push_back(collision_info);
        }
      }

      for (CollisionInfo &info : collision_info_list) {
        if (info.colliding) {
          vrai_collision++;
          //#loop for collision
          // std::cout << "collision: " << vrai_collision << std::endl;
          resolve_collision_with_rotation(info);
          correct_position(info, 1.0 / (correction));
        }
      }
    }
  }
  //#detect collision method
  CollisionInfo detect_collision(int index_box1, int index_box2) {
    CollisionInfo collision_info = CollisionInfo();
    collision_info.index_box1 = index_box1;
    collision_info.index_box2 = index_box2;

    // on recupere les objets et leur caractéristique
    BoxBody box1 = objects[index_box1];
    Vertices vertices_box1 = vertices_list[index_box1];
    Edges edges_box1 = edges_list[index_box1];
    Vector2f position1 = box1.get_position();
    float restitution1 = box1.get_restitution();

    // pareil pour le deuxieme
    BoxBody box2 = objects[index_box2];
    Vertices vertices_box2 = vertices_list[index_box2];
    Edges edges_box2 = edges_list[index_box2];
    Vector2f position2 = box2.get_position();
    float restitution2 = box2.get_restitution();

    // liste des axes
    std::vector<Vector2f>
        axes; // les axes a tester pour le sat (soit ici les cotés des box)

    for (Line edge : edges_box1.get_edges()) {
      Vector2f v = edge.get_vector();
      axes.push_back(v / v.get_norm());
    }

    for (Line edge : edges_box2.get_edges()) {
      Vector2f v = edge.get_vector();
      axes.push_back(v / v.get_norm());
    }

    for (Vector2f axis : axes) // on projette sur chaque axe
    {
      // tableau de taille 2 [min,max] du segment projete
      std::vector<float> projection_box1 =
          project_box_on_axis(index_box1, axis);
      // pareil
      std::vector<float> projection_box2 =
          project_box_on_axis(index_box2, axis);

      // if no collision
      if (projection_box1[1] < projection_box2[0] ||
          projection_box2[1] < projection_box1[0]) {
        collision_info.colliding = false;
        // std::cout << "not colliding! "<< std::endl;
        return collision_info;
      }

      float projection_min = std::min(projection_box1[1], projection_box2[1]);
      float projection_max = std::max(projection_box1[0], projection_box2[0]);
      float penetration_depth = projection_min - projection_max;
      // std::cout << "penetration depth : "<< penetration_depth<< std::endl;
      // on cherche la depth minimale
      if (penetration_depth < collision_info.penetration_depth) {
        collision_info.penetration_depth = penetration_depth;
        // std::cout << "NEW DEPTH"<< std::endl;
        collision_info.set_normal(axis);
      }
    }
    // std::cout << "\x1B[2J\x1B[H" << std::endl; // clear console
    //  on s'assure de toujours pointe de l'objet 1 vers l'objet 2 pour
    //  eviter le bordel apres
    if (collision_info.normal.dot_product(box2.get_center() -
                                          box1.get_center()) < 0) {
      // std::cout << "inverted"<<  std::endl;
      collision_info.normal = -collision_info.normal;
    }
    collision_info.set_normal(collision_info.normal /
                              collision_info.normal.get_norm());

    // relative velocity along collision normal
    // std::cout <<box2.get_velocity().to_string() << "  " <<
    // collision_info.normal.to_string() <<   std::endl;
    collision_info.relative_normal_velocity = collision_info.normal.dot_product(
        box2.get_velocity() - box1.get_velocity());

    collision_info.relative_angular_velocity =
        box2.get_angular_velocity() - box1.get_angular_velocity();

    // std::cout << collision_info.relative_normal_velocity<<  std::endl;
    // si les objets s'eloignent la collision a deja ete resolu
    /* if (collision_info.relative_normal_velocity > 0) {
      // std::cout << "s'eloigne" <<   std::endl;
      collision_info.colliding = false;
      return collision_info;
    } */

    // ceofficient de restitution
    collision_info.restitution = std::min(restitution1, restitution2);

    // calcul des coordonnes des points de collision / il faudrait le fusionner
    // au code reste du code de detection si possible
    for (Line &line1 : edges_box1.get_edges()) {
      for (Line &line2 : edges_box2.get_edges()) {
        std::optional<Vector2f> collision_point = line1.intersection(line2);
        if (collision_point) {
          collision_info.collision_points.push_back(collision_point.value());
        }
      }
    }

    if (collision_info.collision_points.size() == 0) {
      printf("collision without collision points ??");
    }

    collision_info.center_to_col_point1 =
        (collision_info.get_collision_points_center() -
         box1.get_center()); //.get_normalized_normal(); //*
                             // box1.get_angular_velocity();
    collision_info.center_to_col_point2 =
        (collision_info.get_collision_points_center() -
         box2.get_center()); //.get_normalized_normal();// *
                             // box2.get_angular_velocity();

    return collision_info;
  }

  // permet de projeter tous les cotes d une box sur un axe (attention ne marche
  // que avec les polygones convexes)
  std::vector<float> project_box_on_axis(int box_index, Vector2f axis) {
    //-----
    std::vector<Line> edges = edges_list[box_index].get_edges();
    std::vector<float> projection = project_line_on_axis(edges[0], axis);
    float projection_min = projection[0];
    float projection_max = projection[1];

    for (int i = 1; i < edges.size(); i++) {
      // on projete
      projection = project_line_on_axis(edges[i], axis);
      // on regarde si on a un nouveau minimum
      projection_min = std::min(projection_min, projection[0]);
      // pareil pour le max
      projection_max = std::max(projection_max, projection[1]);
    }

    return std::vector<float>{projection_min, projection_max};
  }
  // permet de projeter un segment (ici une line) sur un axe
  std::vector<float> project_line_on_axis(Line line, Vector2f axis) {
    float projection_start =
        line.start_point.dot_product(axis); // on projete le premier point
    float projection_end = line.end_point.dot_product(axis); // le deuxieme

    float projection_min = std::min(projection_start, projection_end);
    float projection_max = std::max(projection_start, projection_end);

    return std::vector<float>{projection_min, projection_max};
  }
  //#resolve collision resolve collision without rotation
  void resolve_collision(CollisionInfo collision_info) {
    BoxBody &box1 = objects[collision_info.index_box1];
    BoxBody &box2 = objects[collision_info.index_box2];

    Vector2f collision_normal = collision_info.normal;
    Vector2f relative_velocity = (box2.get_velocity()) - (box1.get_velocity());

    // si il s'eloigne
    if (relative_velocity.dot_product(collision_normal) >0) {
      // printf("s'eloigne");
      return;
    }
    float linear_impulse = -(1.0 + collision_info.restitution) *
                           relative_velocity.dot_product(collision_normal);
    linear_impulse /= (box1.get_inverse_mass() + box2.get_inverse_mass());
    // adding velocity
    Vector2f impulse_vec = collision_normal * linear_impulse;

    box1.add_to_velocity(-impulse_vec * box1.get_inverse_mass());

    box2.add_to_velocity(impulse_vec * box2.get_inverse_mass());

  }

  //#resolve collision with rotation method
  void resolve_collision_with_rotation(CollisionInfo collision_info) {
    BoxBody &box1 = objects[collision_info.index_box1];
    BoxBody &box2 = objects[collision_info.index_box2];

    Vector2f collision_normal = collision_info.normal;

    std::vector<Vector2f> collision_points = collision_info.collision_points;

    Vector2f col_point = collision_info.get_collision_points_center();

    Vector2f ra = col_point - box1.get_center();
    Vector2f rb = col_point - box2.get_center();

    Vector2f ra_normal = ra.get_normal();
    Vector2f rb_normal = rb.get_normal();

    Vector2f angular_force1 = ra_normal * box1.get_angular_velocity();
    Vector2f angular_force2 = rb_normal * box2.get_angular_velocity();

    Vector2f relativeVelocity = (box2.get_velocity() ) - (box1.get_velocity());

    float velocity_along_normal =
        relativeVelocity.dot_product(collision_normal);

    if (velocity_along_normal > 0) {
      return;
    }

    float dot1 = ra_normal.dot_product(collision_normal);
    float dot2 = rb_normal.dot_product(collision_normal);

    float denom = box1.get_inverse_mass() + box2.get_inverse_mass() +
                  (dot1 * dot1) * box1.get_inverse_inertia() +
                  (dot2 * dot2) * box2.get_inverse_inertia();

    if (std::isnan(denom)) {
      std::cout << "------ALERTE denom-------" << std::endl;
      std::cout << "boxes inv mass "
                << box1.get_inverse_mass() + box2.get_inverse_mass()
                << std::endl;
      std::cout << "dot1 " << dot1 << std::endl;
      std::cout << "dot2 " << dot2 << std::endl;
      std::cout << "box1 inv inertia " << box1.get_inverse_inertia()
                << std::endl;
      std::cout << "box2 inv inertia " << box2.get_inverse_inertia()
                << std::endl;

      std::cout << "ra_normal " << ra_normal.to_string() << std::endl;
      std::cout << "ra_normal " << ra_normal.to_string() << std::endl;

      std::cout << "ra " << ra.to_string() << std::endl;
      std::cout << "rb " << rb.to_string() << std::endl;

      std::cout << "col point " << col_point.to_string() << std::endl;
      std::cout << "col point2 " << rb.to_string() << std::endl;
      return;
    }
    float restitution = collision_info.restitution;
    float j = -(1.0 + restitution) * velocity_along_normal;
    // std::cout<<"j :" << j<< std::endl;
    j /= denom;

    Vector2f impulse = collision_normal * j;

    box1.add_to_velocity(-impulse * box1.get_inverse_mass());

    float new_angular_velocity =
        -ra.cross_product(impulse) * box1.get_inverse_inertia();

    box1.add_to_angular_velocity(new_angular_velocity);


    box2.add_to_velocity(impulse * box2.get_inverse_mass());

    new_angular_velocity =
        rb.cross_product(impulse) * box2.get_inverse_inertia();

    box2.add_to_angular_velocity(new_angular_velocity);
  }
  //correct position method
  void correct_position(CollisionInfo collision_info, float strength) {

    BoxBody &box1 = objects[collision_info.index_box1];
    BoxBody &box2 = objects[collision_info.index_box2];

    if (collision_info.penetration_depth < correction_treshold) {
      return;
    } // si on doit faire qu'une tres faible correction on skip

    /* float position_correction =
        collision_info.penetration_depth -
        correction_treshold /
            (box1.get_inverse_mass() + box2.get_inverse_mass()) *
            correction_percentage * strength; */
    float position_correction = (collision_info.penetration_depth / 2) *
                                strength * correction_percentage;

    if (box1.is_immovable()) {
      box2.add_to_position(collision_info.normal * (position_correction));
    } else if (box2.is_immovable()) {
      box1.add_to_position(collision_info.normal * (-position_correction));
    } else {
      box1.add_to_position(collision_info.normal * (-position_correction) *
                           0.5);
      box2.add_to_position(collision_info.normal * (position_correction) * 0.5);
    }
  }

  // inspired by https://swharden.com/blog/2022-02-01-point-in-rectangle/
  //"How does it works?" section
  std::optional<BoxBody *> get_body_at_position(float a_x, float a_y,
                                                float error_factor = 200) {
    std::optional<BoxBody *> box_at_position = {};
    for (int i = 0; i < edges_list.size(); i++) {
      Edges box_vertices = edges_list[i];
      float total_triangles_area = 0;
      // sum of areas of the 4 rectangles
      for (Line side : box_vertices.get_edges()) {
        float triangle_area =
            (1.0 / 2.0) *
            std::abs(side.start_point.x * (side.end_point.y - a_y) +
                     side.end_point.x * (a_y - side.start_point.y) +
                     a_x * (side.start_point.y - side.end_point.y));
        // std::cout << triangle_area << std::endl;
        total_triangles_area += triangle_area;
      }
      // std::cout <<" " << total_triangles_area <<"<"<< objects[i].get_area()
      // << std::endl;
      if (total_triangles_area <= objects[i].get_area() + error_factor) {
        box_at_position = {&(objects[i])};
        // std::cout << "trouvé " << i << std::endl;
        break;
      }
    }
    return box_at_position;
  }

  std::optional<BoxBody *> get_body_at_position(Vector2f a_point) {
    return get_body_at_position(a_point.x, a_point.y);
  }

  void add_object(BoxBody a_box) {
    a_box.set_id(id_count);
    id_count++;
    objects.push_back(a_box);
  }

  std::vector<BoxBody> get_objects() { return objects; }

  std::vector<Edges> get_edges_vector() { return edges_list; }

  std::vector<Vertices> get_vertices_vector() { return vertices_list; }

  std::vector<CollisionInfo> get_collisions_vector() {
    return collision_info_list;
  }

  void delete_all_objects() { objects.clear(); }

  double get_delta_time() { return delta; }
};

#endif