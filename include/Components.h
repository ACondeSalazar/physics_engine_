#ifndef COMPONENTS_H
#define COMPONENTS_H
#include <vector>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <optional>

struct Vector2
{

    float x;
    float y;

    Vector2()
    {
        x = 0;
        y = 0;
    }

    Vector2(float a_x, float a_y)
    {
        x = a_x;
        y = a_y;
    }

    Vector2 operator+(const Vector2 other) const
    {
        return Vector2(x+other.x, y+other.y);
    }

    Vector2 operator-(const Vector2 other) const
    {
        return Vector2(x-other.x, y-other.y);
    }

    Vector2 operator-() const
    {
        return Vector2(-x,-y);
    }

    Vector2 operator*(const float n) const
    {
        return Vector2(x*n, y*n);
    }

    Vector2 operator/(const float n) const
    {
        return Vector2(x/n, y/n);
    }

    sf::Vector2f get_vector2f(){return sf::Vector2f(x,y);}

    Vector2 get_normal()
    {
        return Vector2(-y, x);
    }

    Vector2 normalized_normal()
    {
        float norm = get_norm();
        return Vector2(y/norm, -x/norm);
    }

    float get_norm()
    {
        return std::sqrt((x*x) + (y*y));
    }


    float dot_product(Vector2 other)
    {
        return (x*other.x) + (y*other.y);
    }

};

struct Vertices
{
    std::vector<Vector2> vertices;

    Vertices()
    {
        vertices = std::vector<Vector2>(0);
    }

    void add_vertex(Vector2 a_vertex)
    {
        vertices.push_back(a_vertex);
    }

    std::vector<Vector2> get_vertices()
    {
        return vertices;
    }

    Vector2 get_vertex(int n)
    {
        return vertices[n];
    }

};

struct Line
{
    Vector2 start_point;
    Vector2 end_point;

    Line(Vector2 a_start_point, Vector2 a_end_point)
    {
        start_point = a_start_point;
        end_point = a_end_point;
    }

    Line(): Line(Vector2(),Vector2()){}

    Vector2 normalized_normal()
    {
        return (end_point-start_point).normalized_normal();
    }

    Vector2 normalized_normal_upward()
    {
        Vector2 v = get_vector();
        return Vector2(-v.y/v.get_norm(), v.x/v.get_norm());
    }

    Vector2 normalized_normal_downward()
    {
        Vector2 v = get_vector();
        return Vector2(v.y/v.get_norm(), -v.x/v.get_norm());
    }

    bool intersect_nul(Line other)
    {
        float orientation1 = orientation(start_point, end_point, other.start_point);
        float orientation2 = orientation(start_point, end_point, other.end_point);
        float orientation3 = orientation(other.start_point, other.end_point, start_point);
        float orientation4 = orientation(other.start_point, other.end_point, end_point);

        if (orientation1 * orientation2 < 0 and orientation3 * orientation4 < 0) //if segment intersect
        {
            float t = orientation3 / orientation(other.start_point, other.end_point, start_point - end_point);
        }
        return true;
    }

    float orientation(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        return (p2.y - p1.y) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.y - p2.y);
    }

    std::optional<Vector2> intersection(Line other)// https://nguyen.univ-tln.fr/share/GeomAlgo/trans_inter.pdf // this(A,B), other(C,B)
    {
        float det = determinant(other);
        if(det == 0){return {};} // segments parallele

        Vector2 A = start_point;
        Vector2 B = end_point;
        Vector2 C = other.start_point;
        Vector2 D = other.end_point;

        float t1 = (  (C.x - A.x)*(C.y - D.y) - (C.x - D.x)*(C.y - A.y)  )/det;
        float t2 = (  (B.x - A.x)*(C.y - A.y) - (C.x - A.x)*(B.y - A.y)  )/det;

        if(t1 > 1 || t1 < 0 || t2 > 1 || t2 < 0){return {};}

        if(t1 == 0){ return A;}
        if(t1 == 1){ return B;}
        if(t2 == 0){ return C;}
        if(t2 == 1){ return D;}

        float x_intersection = A.x + t1*(B.x - A.x);
        float y_intersection = A.y + t1*(B.y - A.y);
        return {Vector2(x_intersection, y_intersection)};
    }

    float determinant(Line other) // https://nguyen.univ-tln.fr/share/GeomAlgo/trans_inter.pdf
    {
        return (end_point.x - start_point.x)*(other.start_point.y - other.end_point.y) - (other.start_point.x - other.end_point.x)*(end_point.y - start_point.y);
    }

    Vector2 get_vector(){return end_point - start_point;} //renvoie le vecteur end - start (le vecteur entre les 2 points)

};

struct Edges
{
    std::vector<Line> edges = std::vector<Line>(0);

    Edges(){}

    Edges(Vertices vertices)
    {
        make_edges(vertices);
    }

    void make_edges(Vertices a_vertices)
    {
        std::vector<Vector2> vertices = a_vertices.get_vertices();
        edges.push_back( Line(vertices[0],vertices[1]) );
        edges.push_back( Line(vertices[1],vertices[2]) );
        edges.push_back( Line(vertices[2],vertices[3]) );
        edges.push_back( Line(vertices[3],vertices[0]) );
    }

    std::vector<Line> get_edges()
    {
        return edges;
    }
};




struct Box
{
    private:
        Vector2 position;
        Vector2 velocity;
        Vector2 size_sides; //longueur des cotés du rectangle
        float angular_velocity;
        float rotation;
        float mass;
        float inverse_mass;
        float restitution;
        bool immovable; //can the rectangle move

    public:
        //constructeurs
        Box(float width, float height)
        {
            position = Vector2();
            velocity = Vector2();
            size_sides = Vector2(width, height);
            angular_velocity = 0;
            rotation = 0;
            set_mass(1);
            restitution = 0;
            immovable = false;
        }

        Box() : Box(200,100){}

        //position
        Vector2 get_position(){ return position;}
        void set_position(float a_x, float a_y)
        {
            position.x = a_x;
            position.y = a_y;
        }
        void set_position(Vector2 a_position)
        {
            position = a_position;
        }

        void add_to_position(float a_dx, float a_dy)
        {
            position.x += a_dx;
            position.y += a_dy;
        }
        void add_to_position(Vector2 a_position)
        {
            position = position + a_position;
        }

        //get center position
        Vector2 get_center()
        {
            return Vector2(position.x+(size_sides.x/2), position.y+(size_sides.y/2));
        }

        //velocity
        Vector2 get_velocity(){ return velocity;}
        void set_velocity(float a_x, float a_y)
        {
            velocity.x = a_x;
            velocity.y = a_y;
        }

        void set_velocity(Vector2 a_velocity)
        {
            velocity = a_velocity;
        }

        void add_to_velocity(float a_dx, float a_dy)
        {
            velocity.x += a_dx;
            velocity.y += a_dy;
        }
        void add_to_velocity(Vector2 a_velocity)
        {
            velocity = velocity + a_velocity;
        }

        //angular_velocity
        float get_angular_velocity(){return angular_velocity;}
        void set_angular_velocity(float a_vel)
        {
            angular_velocity = a_vel;
        }
        void add_to_angular_velocity(float a_vel)
        {
            angular_velocity += a_vel;
        }

        //rotation
        float get_rotation(){return rotation;}
        void set_rotation(float a_angle)
        {
            rotation = a_angle;
        }
        void add_to_rotation(float a_angle)
        {
            rotation += a_angle;
        }

        //size_sides
        Vector2 get_size_sides(){return size_sides;}

        //restitution
        float get_restitution(){return restitution;}
        void set_restitution(float a_restitution)
        {
            restitution = a_restitution;
        }

        //mass
        float get_inverse_mass(){return inverse_mass;}
        float get_mass(){return mass;}
        void set_mass(float a_mass)
        {
            mass = a_mass;
            if(mass <= 0)
            {
                inverse_mass = 0;
            }else{
                inverse_mass = 1/a_mass;
            }

        }

        //vertices
        Vertices get_vertices()
        {
            Vertices vertices = Vertices();

            float width = size_sides.x;
            float height = size_sides.y;

            float width_half = width/2;
            float height_half = height/2;

            //on prend la position du centre
            float center_x = position.x + width_half;
            float center_y = position.y + height_half;

            //position des 4 coins (x,y) relative au centre
            float dx[4] = { -width_half, width_half, width_half, -width_half };
            float dy[4] = { -height_half, -height_half, height_half, height_half };

            float cos_rotation = cos(rotation*3.14159/180);
            float sin_rotation = sin(rotation*3.14159/180);

            float vertice_x;
            float vertice_y;
            for (int i = 0; i < 4; i++)
            {
                vertice_x = center_x + (dx[i] * cos_rotation - dy[i] * sin_rotation);
                vertice_y = center_y + (dx[i] * sin_rotation + dy[i] * cos_rotation);
                Vector2 vertice = Vector2(vertice_x, vertice_y);
                vertices.add_vertex(vertice);
            }

            return vertices;
        }

        //static
        bool is_immovable(){return immovable;}
        void set_immovable(bool b)
        {
            immovable = b;
        }
};

#endif // COMPONENTS_H
