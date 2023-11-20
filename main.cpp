#include <SFML/Graphics.hpp>
#include <iostream>
#include <sstream>
#include "Engine.h"
#include "Components.h"
#include "FPS.h"

int main()
{

    const int HEIGHT = 800;
    const int WIDTH = 1200;

    // Create the main window
    sf::RenderWindow app(sf::VideoMode(WIDTH, HEIGHT), "sim ");

    //app.setFramerateLimit(60);
    FPS fps;
    sf::Clock clock;
    double delta = clock.restart().asSeconds();

    Physics_simulation_engine engine = Physics_simulation_engine(WIDTH, HEIGHT);
    //adding object
    /*
    Box b = Box();
    b.set_position(WIDTH/2, 0);
    b.set_velocity(0, 3);
    engine.add_object(b);

    */
    Box b = Box(WIDTH, 50);
    b.set_position(0, HEIGHT -100);
    b.set_immovable(true);
    //b.set_restitution(0);
    b.set_mass(0);
    engine.add_object(b);

    b = Box(200, 40);
    b.set_position(500, 400);
    b.set_immovable(true);
    b.set_mass(0);
    engine.add_object(b);


    sf::Vector2i mouse_position = sf::Mouse::getPosition(app);
	// Start the game loop
    while (app.isOpen())
    {
        // Process events
        sf::Event event;
        while (app.pollEvent(event))
        {
            // Close window : exit
            if (event.type == sf::Event::Closed){
                std::cout << "EXITING";
                engine.delete_all_objects();
                app.close();
            }else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    mouse_position = sf::Mouse::getPosition(app);
                    Box box_to_place = Box(50, 50);
                    sf::Vector2f size_shape = box_to_place.get_size_sides().get_vector2f();
                    box_to_place.set_position(mouse_position.x-size_shape.x/2, mouse_position.y-size_shape.y/2);
                    //box_to_place.set_rotation(45);
                    box_to_place.set_velocity(0,500);
                    //box_to_place.set_angular_velocity(100);
                    engine.add_object(box_to_place);
                    std::cout << "placed block" << std::endl;
                    std::cout << "Mouse Click Position: " << mouse_position.x << ", " << mouse_position.y << std::endl;
                }
                if (event.mouseButton.button == sf::Mouse::Right) {
                    mouse_position = sf::Mouse::getPosition(app);
                    Box box_to_place = Box(100, 50);
                    sf::Vector2f size_shape = box_to_place.get_size_sides().get_vector2f();
                    box_to_place.set_position(mouse_position.x-size_shape.x/2, mouse_position.y-size_shape.y/2);
                    box_to_place.set_velocity(0,0);
                    engine.add_object(box_to_place);
                    std::cout << "placed block" << std::endl;
                    std::cout << "Mouse Click Position: " << mouse_position.x << ", " << mouse_position.y << std::endl;
                }
            }

        }

        //update
        engine.update(delta);

        // Clear screen
        app.clear();

        //draw objects
        std::vector<Box> objects = engine.get_objects();
        std::vector<Edges> edges_vector = engine.get_edges_vector();
        std::vector<Vertices> vertices_vector = engine.get_vertices_vector();
        for(int i = 0; i < objects.size(); i++) //objects est une liste de pointeur
        {
                Box box = objects[i];
                Vertices vertices = vertices_vector[i];
                Edges edges = edges_vector[i];

                //dessiner rectangle
                sf::RectangleShape rectangle;

                Vector2 position = box.get_position();
                Vector2 size_shape = box.get_size_sides();

                rectangle.setPosition(position.x+size_shape.x/2, position.y+size_shape.y/2);
                rectangle.setSize(size_shape.get_vector2f());
                rectangle.setOrigin((int)size_shape.x/2, (int)size_shape.y/2);
                rectangle.setRotation(box.get_rotation());

                rectangle.setFillColor(sf::Color::White);

                //app.draw(rectangle);

                //dessiner les sommets du rectangle
                for(Vector2 vertex : vertices.get_vertices())
                {
                    sf::CircleShape circle;

                    circle.setPosition(vertex.x, vertex.y);
                    int radius = 3;
                    circle.setRadius(radius);
                    circle.setOrigin(radius,radius);
                    circle.setFillColor(sf::Color::Red);
                    app.draw(circle);
                }


                //dessinner les cotés
                for(Line line : edges.get_edges()) //pour chaque coté d'une box
                {
                    sf::VertexArray ligne(sf::Lines, 2);
                    ligne[0].position = line.start_point.get_vector2f();
                    ligne[1].position = line.end_point.get_vector2f();
                    ligne[0].color = sf::Color::Green;
                    ligne[1].color = sf::Color::Green;
                    app.draw(ligne);
                }

                //centre du rectangle
                sf::CircleShape circle;

                circle.setPosition(position.x + size_shape.x/2, position.y + size_shape.y/2);
                circle.setPosition(box.get_center().get_vector2f());
                int radius = 3;
                circle.setRadius(radius);
                circle.setOrigin(radius,radius);
                circle.setFillColor(sf::Color::Yellow);
                app.draw(circle);

                //dessiner velocite
                Vector2 center = box.get_center();
                sf::VertexArray ligne(sf::Lines, 2);
                ligne[0].position = center.get_vector2f();
                ligne[1].position = (center+box.get_velocity()*5).get_vector2f();
                ligne[0].color = sf::Color::Blue;
                ligne[1].color = sf::Color::Blue;
                app.draw(ligne);

        }
        //draw collision normal
        std::vector<CollisionInfo> collisions_vector = engine.get_collisions_vector();
        for(int i = 0; i< collisions_vector.size(); i++)
        {
            CollisionInfo info = collisions_vector[i];
            if(info.colliding)
            {
                Vector2 normal = info.normal * info.penetration_depth * 10;

                Vector2 center1 = objects[info.index_box1].get_center();
                Vector2 center2 = objects[info.index_box2].get_center();

                /*
                sf::VertexArray ligne1(sf::Lines, 2);
                ligne1[0].position = center1.get_vector2f();
                ligne1[1].position = (center1+normal).get_vector2f();
                ligne1[0].color = sf::Color::Yellow;
                ligne1[1].color = sf::Color::Yellow;
                app.draw(ligne1);


                sf::VertexArray ligne2(sf::Lines, 2);
                ligne2[0].position = center2.get_vector2f();
                ligne2[1].position = (center2-normal).get_vector2f();
                ligne2[0].color = sf::Color::Yellow;
                ligne2[1].color = sf::Color::Yellow;
                app.draw(ligne2);
                */

                sf::VertexArray ligne1(sf::Lines, 2);
                ligne1[0].position = center1.get_vector2f();
                ligne1[1].position = (center1+normal).get_vector2f();
                ligne1[0].color = sf::Color::Yellow;
                ligne1[1].color = sf::Color::Yellow;
                app.draw(ligne1);

                for(Vector2& collision_point : info.collision_points)
                {
                    //centre du rectangle
                    sf::CircleShape circle;

                    circle.setPosition(collision_point.x, collision_point.y);
                    int radius = 4;
                    circle.setRadius(radius);
                    circle.setOrigin(radius,radius);
                    circle.setFillColor(sf::Color::Blue);
                    app.draw(circle);
                }

            }
        }

        // Update the window
        app.display();

        //show fps //trouver sur internet je sais plus ou
        fps.update();
        std::ostringstream fpsString;
        fpsString << fps.getFPS();
        app.setTitle("fps: " + fpsString.str() + "/ objects: " + to_string(objects.size()) +"/ delta: " + to_string(engine.get_delta_time())  );

        delta = clock.restart().asSeconds();
    }

    return EXIT_SUCCESS;
}
