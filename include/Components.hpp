#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <optional>
#include <string.h>

class Vector2f {

public:
  float x;
  float y;

  Vector2f() {
    x = 0;
    y = 0;
  }

  Vector2f(float a_x, float a_y) {
    x = a_x;
    y = a_y;
  }

  Vector2f operator+(const Vector2f other) const {
    return Vector2f(x + other.x, y + other.y);
  }

  Vector2f operator-() const { return Vector2f(-x, -y); }

  Vector2f operator-(const Vector2f other) const {
    return Vector2f(x - other.x, y - other.y);
  }

  Vector2f operator*(const float n) const { return Vector2f(x * n, y * n); }

  Vector2f operator/(const float n) const { return Vector2f(x / n, y / n); }

  Vector2f get_normal() { return Vector2f(-y, x); }

  float get_norm() { return sqrt((x * x) + (y * y)); }

	Vector2f get_normalized_normal(){ return get_normal()/get_norm();}

  float dot_product(Vector2f other) { return (x * other.x) + (y * other.y); }

  sf::Vector2f convert_to_sfml() { return sf::Vector2f(x, y); }

  std::string to_string(){ return "(" + std::to_string((int)x) + "," + std::to_string((int)y) + ")";}

};

class Vertices {
private:
  std::vector<Vector2f> vertices;

public:
  void add_vertex(Vector2f a_vertex) { vertices.push_back(a_vertex); }

  std::vector<Vector2f> get_vertices() { return vertices; }

  Vector2f get_vertex(int n) { return vertices[n]; }
};

// represent the line between 2 points (2 Vector2f)
struct Line {
  Vector2f start_point;
  Vector2f end_point;

  Line(Vector2f a_start_point, Vector2f a_end_point) {
    start_point = a_start_point;
    end_point = a_end_point;
  }

  Line() : Line(Vector2f(), Vector2f()) {}

  Vector2f normalized_normal() {
    return (end_point - start_point).get_normalized_normal();
  }

  Vector2f normalized_normal_upward() {
    Vector2f v = get_vector();
    return Vector2f(-v.y / v.get_norm(), v.x / v.get_norm());
  }

  Vector2f normalized_normal_downward() {
    Vector2f v = get_vector();
    return Vector2f(v.y / v.get_norm(), -v.x / v.get_norm());
  }

  float orientation(Vector2f p1, Vector2f p2, Vector2f p3) {
    return (p2.y - p1.y) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.y - p2.y);
  }

  // https://nguyen.univ-tln.fr/share/GeomAlgo/trans_inter.pdf
  // this(A,B), other(C,B)
  std::optional<Vector2f> intersection(Line other){
    float det = determinant(other);
    if (det == 0) {
      return {};
    } // segments parallele

    Vector2f A = start_point;
    Vector2f B = end_point;
    Vector2f C = other.start_point;
    Vector2f D = other.end_point;

    float t1 = ((C.x - A.x) * (C.y - D.y) - (C.x - D.x) * (C.y - A.y)) / det;
    float t2 = ((B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y)) / det;

    if (t1 > 1 || t1 < 0 || t2 > 1 || t2 < 0) {
      return {};
    }

    if (t1 == 0) {
      return A;
    }
    if (t1 == 1) {
      return B;
    }
    if (t2 == 0) {
      return C;
    }
    if (t2 == 1) {
      return D;
    }

    float x_intersection = A.x + t1 * (B.x - A.x);
    float y_intersection = A.y + t1 * (B.y - A.y);
    return {Vector2f(x_intersection, y_intersection)};
  }

  // https://nguyen.univ-tln.fr/share/GeomAlgo/trans_inter.pdf
  float determinant(Line other) {
    return (end_point.x - start_point.x) *
               (other.start_point.y - other.end_point.y) -
           (other.start_point.x - other.end_point.x) *
               (end_point.y - start_point.y);
  }

  Vector2f get_vector() {
    return end_point - start_point;
  } // renvoie le vecteur end - start (le vecteur entre les 2 points)
};

struct Edges {
  std::vector<Line> edges = std::vector<Line>(0);

  Edges() {}

  Edges(Vertices vertices) { make_edges(vertices); }

  void make_edges(Vertices a_vertices) {
    std::vector<Vector2f> vertices = a_vertices.get_vertices();
    edges.push_back(Line(vertices[0], vertices[1]));
    edges.push_back(Line(vertices[1], vertices[2]));
    edges.push_back(Line(vertices[2], vertices[3]));
    edges.push_back(Line(vertices[3], vertices[0]));
  }

  std::vector<Line> get_edges() { return edges; }
};

class Entity {
protected:
  Vector2f position;
  Vector2f velocity;
  float angular_velocity;
  float rotation;
  float mass;
  float inverse_mass;
  float restitution;
  bool immovable;
  

  Entity() {
    position = Vector2f();
    velocity = Vector2f();
    angular_velocity = 0;
    rotation = 0;
    set_mass(1);
    restitution = 0.5;
    immovable = false;
	color = sf::Color::Black;
  }

public:
	sf::Color color;
	int id;
  void set_id(int a_id){ id = a_id;}
  Vector2f get_position() { return position; }
  void set_position(float a_x, float a_y) {
    position.x = a_x;
    position.y = a_y;
  }
  void set_position(Vector2f a_position) { position = a_position; }

  void add_to_position(float a_dx, float a_dy) {
    position.x += a_dx;
    position.y += a_dy;
  }
  void add_to_position(Vector2f a_position) {
    position = position + a_position;
  }

  // velocity
  Vector2f get_velocity() { return velocity; }
  void set_velocity(float a_x, float a_y) {
    velocity.x = a_x;
    velocity.y = a_y;
  }

  void set_velocity(Vector2f a_velocity) { velocity = a_velocity; }

  void add_to_velocity(float a_dx, float a_dy) {
    velocity.x += a_dx;
    velocity.y += a_dy;
  }
  void add_to_velocity(Vector2f a_velocity) {
    velocity = velocity + a_velocity;
  }

  // angular_velocity
  float get_angular_velocity() { return angular_velocity; }
  void set_angular_velocity(float a_vel) { angular_velocity = a_vel; }
  void add_to_angular_velocity(float a_vel) { angular_velocity += a_vel; }

  // rotation
  float get_rotation() { return rotation; }
  void set_rotation(float a_angle) { rotation = a_angle; }
  void add_to_rotation(float a_angle) { rotation += a_angle; }

  // restitution
  float get_restitution() { return restitution; }
  void set_restitution(float a_restitution) { restitution = a_restitution; }

  // mass
  float get_inverse_mass() { return immovable? 0 : inverse_mass; }
  float get_mass() { return immovable? 0: mass; }
  void set_mass(float a_mass) {
    mass = a_mass;
    if (mass <= 0) {
      inverse_mass = 0;
    } else {
      inverse_mass = 1 / a_mass;
    }
  }

  // static
  bool is_immovable() { return immovable; }
  void set_immovable(bool b) { immovable = b; }

  virtual Vector2f get_center() = 0;
};

class BoxBody : public Entity {
private:
  Vector2f size_sides; // longueur des cotés du rectangle

public:
  BoxBody(float a_width, float a_height) : Entity() {
    size_sides = Vector2f(a_width, a_height);
  }

  BoxBody() : BoxBody(200, 100) {}

  Vector2f get_center() {
    return Vector2f(position.x + (size_sides.x / 2),
                    position.y + (size_sides.y / 2));
  }

  float get_area(){
	return size_sides.x*size_sides.y;
  }

	Vector2f get_size_sides(){return size_sides;}

  // vertices
  Vertices get_vertices() {
    Vertices vertices = Vertices();

    float width = size_sides.x;
    float height = size_sides.y;

    float width_half = width / 2;
    float height_half = height / 2;

    // on prend la position du centre
    float center_x = position.x + width_half;
    float center_y = position.y + height_half;

    // position des 4 coins (x,y) relative au centre
    float dx[4] = {-width_half, width_half, width_half, -width_half};
    float dy[4] = {-height_half, -height_half, height_half, height_half};

    float cos_rotation = cos(rotation * 3.14159 / 180);
    float sin_rotation = sin(rotation * 3.14159 / 180);

    float vertice_x;
    float vertice_y;
    for (int i = 0; i < 4; i++) {
      vertice_x = center_x + (dx[i] * cos_rotation - dy[i] * sin_rotation);
      vertice_y = center_y + (dx[i] * sin_rotation + dy[i] * cos_rotation);
      Vector2f vertice = Vector2f(vertice_x, vertice_y);
      vertices.add_vertex(vertice);
    }

    return vertices;
  }
};

struct CollisionInfo
{
        int index_box1;
        int index_box2;
        Vector2f normal; //axis along which the 2 objects collide NOT A UNIT VECTOR
		//Vector2f normal_unit; // unit vector of the 
        float relative_normal_velocity; //relative velocity along the collision normal hence why its a scalar
        float relative_angular_velocity;
        float penetration_depth;
        float restitution;
        float linear_impulse;
        std::vector<Vector2f> collision_points;
        bool colliding;


        CollisionInfo()
        {
            normal = Vector2f();
            relative_normal_velocity = 0;
            penetration_depth = 10000;
            restitution = 0;
            linear_impulse = 0;
            colliding = true;
        }

        //normal
        Vector2f get_normal(){return normal;}
        void set_normal(Vector2f a_normal)
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

#endif