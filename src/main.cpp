#include "../include/Components.hpp"
#include "../include/Engine.hpp"
#include "../include/Fps.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <optional>
#include <ostream>
#include <sstream>
#include <vector>
// #include <random>

int main(int argc, char *argv[]) {

  if (argc != 3) {
    std::cout << "usage is: " << argv[0] << " width height" << std::endl;
    return 1;
  }

  const int WIDTH = atoi(argv[1]);
  const int HEIGHT = atoi(argv[2]);

  srand(time(NULL));

  // main window
  sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "simulation");

  window.setFramerateLimit(60);
  FPS fps;
  sf::Clock clock;
  double delta = clock.restart().asSeconds();

  Physics_simulation_engine engine = Physics_simulation_engine(WIDTH, HEIGHT);
  engine.update(0.0016);

  BoxBody b = BoxBody(WIDTH, 50);
  b.set_position(0, HEIGHT - 100);
  b.set_immovable(true);
  b.set_restitution(.1);
  b.set_mass(0);
  engine.add_object(b);

  b = BoxBody(200, 40);
  b.set_position(500, 400);
  b.set_immovable(true);
  b.set_mass(0);
  b.set_rotation(135);
  engine.add_object(b);

  for (int i = 0; i < 0; i++) {
    float r_x = rand() % WIDTH;
    float r_y = rand() % HEIGHT;
    BoxBody r_box = BoxBody(rand() % 40 + 30, rand() % 40 + 30);
    r_box.set_position(r_x, r_y);
    r_box.set_rotation(rand() % 180);
    r_box.set_mass(rand() % 10 + 1);
    engine.add_object(r_box);
  }

  sf::Vector2i mouse_position = sf::Mouse::getPosition(window);

  // used for drawing a box
  bool user_drawing = false;
  sf::Vector2i user_drawing_start;
  sf::Vector2i user_drawing_end;

  bool user_dragging_box = false;
  BoxBody *selected_box;

  bool debug = false;
  bool nice_view = true;
  sf::Color background_color =
      !nice_view ? sf::Color::Black : sf::Color(100, 100, 100);

  while (window.isOpen()) {
    // Process events
    mouse_position = sf::Mouse::getPosition(window);
    user_drawing_end = mouse_position;
    sf::Event event;
    while (window.pollEvent(event)) {
      // Close window : exit
      if (event.type == sf::Event::Closed) {
        std::cout << "EXITING" << std::endl;
        engine.delete_all_objects();
        window.close();
      } else if (event.type == sf::Event::MouseButtonPressed) {

        if (event.mouseButton.button == sf::Mouse::Left) {
          user_drawing = true;
          user_drawing_start = mouse_position;
          // std::cout << "start drawing" << std::endl;
        }

        if (event.mouseButton.button == sf::Mouse::Right) {
          //
        }

      } else if (event.type == sf::Event::MouseButtonReleased) {
        // create user drawn rectangle when button released
        if (event.mouseButton.button == sf::Mouse::Left) {
          // std::cout << "end drawing" << std::endl;
          user_drawing_end = mouse_position;
          if (user_drawing_end.x < user_drawing_start.x &&
              user_drawing_end.y < user_drawing_start.y) {
            user_drawing_end = user_drawing_start;
            user_drawing_start = mouse_position;
          } else if (user_drawing_end.x < user_drawing_start.x) {
            int decalage = user_drawing_start.x - user_drawing_end.x;
            user_drawing_start.x -= decalage;
            user_drawing_end.x += decalage;
          } else if (user_drawing_start.y > user_drawing_end.y) {
            int decalage = user_drawing_start.y - user_drawing_end.y;
            user_drawing_start.y -= decalage;
            user_drawing_end.y += decalage;
          }
          float new_box_width =
              std::abs(user_drawing_start.x - user_drawing_end.x);
          float new_box_height =
              std::abs(user_drawing_start.y - user_drawing_end.y);
          if (new_box_width > 20 && new_box_height > 20) {
            BoxBody new_box = BoxBody(new_box_width, new_box_height);
            new_box.set_position(user_drawing_start.x, user_drawing_start.y);
            new_box.set_velocity(Vector2f(0, 0));
            new_box.set_mass(rand() % 100 + 1);
            new_box.set_rotation(0);
            engine.add_object(new_box);
          }
          user_drawing = false;
        }
        if (event.mouseButton.button == sf::Mouse::Right) {
          user_dragging_box = false;
          // std::cout << "released" << std::endl;
        }
      } else if (sf::Mouse::isButtonPressed(
                     sf::Mouse::Right)) { // search and start dragging box
                                          // around
        if (!user_dragging_box) {
          std::optional<BoxBody *> selection =
              engine.get_body_at_position(mouse_position.x, mouse_position.y);
          if (selection.has_value()) {
            user_dragging_box = true;
            selected_box = selection.value();
            // std::cout << "dragging" << std::endl;
          }
        }
      } else if (event.type == sf::Event::KeyPressed &&
                 event.key.code == sf::Keyboard::C) {
        engine.delete_all_objects();
      }
    }

    // updating
    engine.update(delta);

    window.clear(background_color);

    // draw objects
    std::vector<BoxBody> objects = engine.get_objects();
    std::vector<Edges> edges_vector = engine.get_edges_vector();
    std::vector<Vertices> vertices_vector = engine.get_vertices_vector();

    // draw user created object
    if (user_drawing) {
      float new_box_width = std::abs(user_drawing_start.x - user_drawing_end.x);
      float new_box_height =
          std::abs(user_drawing_start.y - user_drawing_end.y);
      // Draw the outline of the rectangle
      sf::VertexArray lines(sf::LinesStrip, 5);
      lines[0].position =
          sf::Vector2f(user_drawing_start.x, user_drawing_start.y);
      lines[1].position =
          sf::Vector2f(user_drawing_end.x, user_drawing_start.y);
      lines[2].position = sf::Vector2f(user_drawing_end.x, user_drawing_end.y);
      lines[3].position =
          sf::Vector2f(user_drawing_start.x, user_drawing_end.y);
      lines[4].position =
          sf::Vector2f(user_drawing_start.x, user_drawing_start.y);

      lines[0].color = sf::Color::Green;
      lines[1].color = sf::Color::Green;
      lines[2].color = sf::Color::Green;
      lines[3].color = sf::Color::Green;
      lines[4].color = sf::Color::Green;

      window.draw(lines);
    }

    // drag box around
    if (user_dragging_box) {
      Vector2f velocity_to_add;
      // selected_box = objects[0];
      velocity_to_add.x = (mouse_position.x - selected_box->get_center().x);
      velocity_to_add.y = (mouse_position.y - selected_box->get_center().y);
      selected_box->set_velocity(velocity_to_add);
    }

    for (int i = 0; i < objects.size(); i++) {
      // objects[i].add_to_velocity(30,150);
      BoxBody box = objects[i];
      Vertices vertices = vertices_vector[i];
      Edges edges = edges_vector[i];

      Vector2f position = box.get_position();
      Vector2f size_shape = box.get_size_sides();

      if (nice_view) {
        // dessiner rectangle
        sf::RectangleShape rectangle;

        rectangle.setPosition(position.x + size_shape.x / 2,
                              position.y + size_shape.y / 2);
        rectangle.setSize(size_shape.convert_to_sfml());
        rectangle.setOrigin((int)size_shape.x / 2, (int)size_shape.y / 2);
        rectangle.setRotation(box.get_rotation());
        int mass_color = 255 / box.get_mass();
        rectangle.setFillColor(
            sf::Color(mass_color, mass_color, mass_color, 255));
        rectangle.setOutlineThickness(2); // Set outline thickness
        if (box.is_immovable()) {
          rectangle.setOutlineColor(sf::Color::Black); // Set outline color
        } else {
          rectangle.setOutlineColor(sf::Color::White); // Set outline color
        }

        window.draw(rectangle);
      }

      if (debug) {
        // window.clear();
        //  dessiner les sommets du rectangle
        for (Vector2f vertex : vertices.get_vertices()) {
          sf::CircleShape circle;

          circle.setPosition(vertex.x, vertex.y);
          int radius = 3;
          circle.setRadius(radius);
          circle.setOrigin(radius, radius);
          circle.setFillColor(sf::Color::Red);
          window.draw(circle);
        }

        // dessinner les cot�s
        for (Line line : edges.get_edges()) // pour chaque cot� d'une box
        {
          sf::VertexArray ligne(sf::Lines, 2);
          ligne[0].position = line.start_point.convert_to_sfml();
          ligne[1].position = line.end_point.convert_to_sfml();
          ligne[0].color = sf::Color::Green;
          ligne[1].color = sf::Color::Green;
          window.draw(ligne);
        }

        // centre du rectangle
        sf::CircleShape circle;

        circle.setPosition(position.x + size_shape.x / 2,
                           position.y + size_shape.y / 2);
        circle.setPosition(box.get_center().convert_to_sfml());
        int radius = 3;
        circle.setRadius(radius);
        circle.setOrigin(radius, radius);
        circle.setFillColor(sf::Color::Yellow);
        window.draw(circle);

        // dessiner velocite
        Vector2f center = box.get_center();
        sf::VertexArray ligne(sf::Lines, 2);
        ligne[0].position = center.convert_to_sfml();
        ligne[1].position = (center + box.get_velocity() / 3).convert_to_sfml();
        ligne[0].color = sf::Color::Blue;
        ligne[1].color = sf::Color::Blue;
        window.draw(ligne);

        // draw collision normal
        std::vector<CollisionInfo> collisions_vector =
            engine.get_collisions_vector();
        for (int i = 0; i < collisions_vector.size(); i++) {
          CollisionInfo info = collisions_vector[i];
          if (info.colliding) {
            Vector2f normal = info.normal * info.penetration_depth * 5;

            Vector2f center1 = objects[info.index_box1].get_center();
            Vector2f center2 = objects[info.index_box2].get_center();

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
            ligne1[0].position = center1.convert_to_sfml();
            ligne1[1].position = (center1 + normal).convert_to_sfml();
            ligne1[0].color = sf::Color::Yellow;
            ligne1[1].color = sf::Color::Yellow;
            window.draw(ligne1);

            // points de collision
            for (Vector2f &collision_point : info.collision_points) {
              // centre du rectangle
              sf::CircleShape circle;

              circle.setPosition(collision_point.x, collision_point.y);
              int radius = 4;
              circle.setRadius(radius);
              circle.setOrigin(radius, radius);
              circle.setFillColor(sf::Color::Blue);
              window.draw(circle);
            }
          }
        }
      }
    }

    // Update the window
    window.display();

    // show fps //trouver sur internet je sais plus ou
    fps.update();
    std::ostringstream fpsString;
    fpsString << fps.getFPS();
    window.setTitle("fps: " + fpsString.str() +
                    "/ objects: " + std::to_string(objects.size()) +
                    "/ delta: " + std::to_string(delta));
    delta = clock.restart().asSeconds();
  }
}