#include "../_deps/imgui-sfml-src/imgui-SFML.h"
#include "../_deps/imgui-src/imgui.h"
#include "../include/Components.hpp"
#include "../include/Engine.hpp"
#include "../include/Fps.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <optional>
#include <ostream>
#include <sstream>
#include <string.h>
#include <vector>
// #include <random>

int main(int argc, char *argv[]) {

  if (argc != 3) {
    std::cout << "usage is: " << argv[0] << " width height" << std::endl;
    return 1;
  }

  const int WINDOW_WIDTH = atoi(argv[1]);
  const int WINDOW_HEIGHT = atoi(argv[2]);

  const int MENU_WIDTH = WINDOW_WIDTH * 0.25;
  const int MENU_HEIGHT = WINDOW_HEIGHT / 2;

  const int SIM_WIDTH = WINDOW_WIDTH;
  const int SIM_HEIGHT = WINDOW_HEIGHT;

  // creating color palette ofr the objects
  std::vector<sf::Color> color_palette;
  color_palette.push_back(sf::Color(0, 175, 185));
  color_palette.push_back(sf::Color(253, 252, 220));
  color_palette.push_back(sf::Color(181, 101, 118));
  color_palette.push_back(sf::Color(240, 113, 103));

  float gravity[2] = {0, 0};
  float gravity_max = 1000;
  float gravity_min = 0;

  srand(time(NULL));

  // Init of all components
  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
                          "simulation");

  window.setFramerateLimit(60);
  FPS fps;
  sf::Clock clock;
  double delta = clock.restart().asSeconds();

  ImGui::SFML::Init(window);

  Physics_simulation_engine engine =
      Physics_simulation_engine(SIM_WIDTH, SIM_HEIGHT);
  engine.update(0.0016);

  // placing initial objects on screen
  BoxBody b = BoxBody(SIM_WIDTH - 200, 75);
  b.set_position(100, SIM_HEIGHT - 100);
  b.set_immovable(true);
  b.set_restitution(.25);
  b.set_mass(0);
  b.color = color_palette[0];
  engine.add_object(b);

  b = BoxBody(200, 40);
  b.set_position(500, 400);
  b.set_immovable(true);
  b.set_mass(0);
  b.set_rotation(135);
  b.color = color_palette[1];
  engine.add_object(b);

  for (int i = 0; i < 10; i++) {
    float r_x = rand() % SIM_WIDTH;
    float r_y = rand() % SIM_HEIGHT;
    BoxBody r_box = BoxBody(rand() % 40 + 30, rand() % 40 + 30);
    r_box.set_position(r_x, r_y);
    r_box.set_rotation(rand() % 180);
    r_box.set_mass(rand() % 10 + 1);
    r_box.color = color_palette[rand() % color_palette.size()];
    engine.add_object(r_box);
  }

  // used for drawing a box
  bool user_drawing = false;
  sf::Vector2i user_drawing_start;
  sf::Vector2i user_drawing_end;

  bool user_dragging_box = false;
  BoxBody *selected_box = nullptr; //&(engine.get_objects()[0]);

  bool debug = false;
  bool simulation_paused = false;

  std::vector<float> update_duration_history;

  while (window.isOpen()) {
    // Process events
    sf::Vector2i mouse_position = sf::Mouse::getPosition(window);
    user_drawing_end = mouse_position;
    sf::Event event;
    while (window.pollEvent(event)) {
      ImGui::SFML::ProcessEvent(window, event);
      // Close window : exit
      if (event.type == sf::Event::Closed) {
        std::cout << "EXITING" << std::endl;
        engine.delete_all_objects();
        window.close();
      } else if (ImGui::GetIO().WantCaptureMouse) {

      } else {
        if (event.type == sf::Event::MouseButtonPressed) {

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
              new_box.color = color_palette[rand() % color_palette.size()];
              engine.add_object(new_box);
              engine.update(0);
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
          selected_box = nullptr;
        }
      }
    }

    auto start = std::chrono::steady_clock::now();
    if (!simulation_paused) {
      // updating
      engine.update(delta);
    }
    auto update_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start);

    if (update_duration_history.size() < 250) {
      update_duration_history.push_back(update_duration.count());
    } else {
      update_duration_history.erase(update_duration_history.begin());
      update_duration_history.push_back(update_duration.count());
    }

    start = std::chrono::steady_clock::now(); // render time start

    // background color is black if debug mode
    sf::Color background_color =
        debug ? sf::Color::Black : sf::Color(100, 100, 100);
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
    if (user_dragging_box && selected_box != nullptr) {
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

      if (!debug) {
        // dessiner rectangle
        sf::RectangleShape rectangle;

        rectangle.setPosition(position.x + size_shape.x / 2,
                              position.y + size_shape.y / 2);
        rectangle.setSize(size_shape.convert_to_sfml());
        rectangle.setOrigin((int)size_shape.x / 2, (int)size_shape.y / 2);
        rectangle.setRotation(box.get_rotation());
        int mass_color = 255 / box.get_mass();
        rectangle.setFillColor(box.color);
        rectangle.setOutlineThickness(2);
        if (selected_box != nullptr && box.id == selected_box->id) {
          rectangle.setOutlineColor(sf::Color::Red);
        } else if (box.is_immovable()) {
          rectangle.setOutlineColor(sf::Color::Black);
        } else {
          rectangle.setOutlineColor(sf::Color::White);
        }

        window.draw(rectangle);
      }

      else {
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
    auto render_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
    fps.update();
    std::ostringstream fpsString;
    fpsString << fps.getFPS();
    window.setTitle("fps: " + fpsString.str() +
                    "/ objects: " + std::to_string(objects.size()) +
                    "/ delta: " + std::to_string(delta));
    delta = clock.getElapsedTime().asSeconds();

    // drawing gui
    ImGui::SFML::Update(window, clock.restart());
    //ImGui::ShowDemoWindow();
    ImGui::Begin("Sim Settings");

    if (ImGui::BeginTabBar("Navigation")) {

      if (ImGui::BeginTabItem("Settings")) {
        if (ImGui::Button("Pause/Play")) {
          simulation_paused = !simulation_paused;
        }

        ImGui::Text("fps: %d", fps.getFPS());
        ImGui::PlotLines("update time", update_duration_history.data(),
                         update_duration_history.size());
		float avg_update_time = 0;
		for (int i = 0; i < update_duration_history.size(); i++){avg_update_time+= update_duration_history[i];}
		avg_update_time /= update_duration_history.size()*1000;
        ImGui::Text("average update duration : %.2f ms", avg_update_time);
        ImGui::Text("render duration : %lu ms", render_duration.count());
        ImGui::Separator();
        ImGui::SliderInt("Accuracy", &(engine.accuracy), 1, 50);
        ImGui::SliderFloat("Position correction treshold",
                           &(engine.correction_treshold), 0, 20);
        ImGui::SliderFloat("Position correction strength",
                           &(engine.correction_percentage), 0, 2);

        ImGui::SliderScalarN("Gravity(x,y)", ImGuiDataType_Float, gravity, 2,
                             &gravity_min, &gravity_max, "%.1f");
        engine.gravity_force = Vector2f(gravity[0], gravity[1]);
        ImGui::Checkbox("Debug view", &debug);
        ImGui::EndTabItem();
      }

      if (ImGui::BeginTabItem("Inspector")) {
        if (selected_box != nullptr) {
          ImGui::Text("Object ID : %d", selected_box->id);
          ImGui::Text("position : %s",
                      selected_box->get_position().to_string().c_str());
          ImGui::Text("velocity : %s",
                      selected_box->get_velocity().to_string().c_str());
          ImGui::Separator();
		  ImGui::Text("Edit properties :");

			bool is_static = selected_box->is_immovable();
          ImGui::Checkbox("static", &is_static);
		  //on ne modifie le tag static que si changement
		  if(is_static != selected_box->is_immovable()){ 
			selected_box->set_immovable(is_static);
		  }
          

		if(!is_static){
			int mass = selected_box->get_mass();
		  ImGui::SliderInt("Mass", &(mass), 1, 500);
		  selected_box->set_mass(mass);

		}
		  float restitution = selected_box->get_restitution();
		  ImGui::SliderFloat("restitution", &(restitution), 0, 1);
		  selected_box->set_restitution(restitution);
		  
        } else {
          ImGui::Text("Right click(or hold) to select an object");
        }
        ImGui::EndTabItem();
      }

      if (ImGui::BeginTabItem("Help")) {
        ImGui::BulletText("Hold Right click to select and drag object");
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();

    ImGui::End();

    ImGui::SFML::Render(window);
    // Update the window

    window.display();

    // show fps //trouver sur internet je sais plus ou
  }
}