
#ifndef PATH_FINDING_MANAGER_H
#define PATH_FINDING_MANAGER_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>
#include <iostream>
#include <SFML/Graphics.hpp>
#include "node.h"
#include "edge.h"
#include "graph.h"
#include "window_manager.h"

enum Algorithm {
    Dijkstra,
    AStar,
    Greedy
};

class PathFindingManager {
public:
    PathFindingManager(WindowManager* wm) : window_manager(wm), src(nullptr), dest(nullptr) {}

    void set_source(Node* node) {
        if (src) {
            src->radius = 2.0f;
            src->color = sf::Color::White;
        }
        src = node;
        if (src) {
            src->radius = 10.0f;
            src->color = sf::Color::Green;
        }
    }

    void set_destination(Node* node) {
        if (dest) {
            dest->radius = 2.0f;
            dest->color = sf::Color::White;
        }
        dest = node;
        if (dest) {
            dest->radius = 10.0f;
            dest->color = sf::Color::Cyan;
        }
    }

    void exec(Graph& graph, Algorithm algo) {
        if (src == nullptr || dest == nullptr) return;

        visited_edges.clear();
        path.clear();

        switch (algo) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case Greedy:
                greedy(graph);
                break;
        }
    }

    void draw(bool show_visited_edges) {
        sf::RenderWindow& window = window_manager->get_window();
        if (show_visited_edges) {
            for (auto& line : visited_edges)
                window.draw(line);
        }
        for (auto& line : path)
            window.draw(line);
    }

    void reset() {
        if (src) {
            src->radius = 2.0f;
            src->color = sf::Color::White;
        }
        if (dest) {
            dest->radius = 2.0f;
            dest->color = sf::Color::White;
        }
        src = nullptr;
        dest = nullptr;
        path.clear();
        visited_edges.clear();
    }

    Node* src = nullptr;
    Node* dest = nullptr;

private:
    WindowManager* window_manager;
    std::vector<sf::VertexArray> path;
    std::vector<sf::VertexArray> visited_edges;

    float heuristic(Node* a, Node* b) {
        float dx = a->coord.x - b->coord.x;
        float dy = a->coord.y - b->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void draw_edge(Node* from, Node* to, sf::Color color) {
        if (!from || !to) return;
        sf::VertexArray line(sf::Lines, 2);
        line[0].position = from->coord;
        line[1].position = to->coord;
        line[0].color = color;
        line[1].color = color;
        visited_edges.push_back(line);
    }

    void set_final_path(const std::unordered_map<Node*, Node*>& prev) {
        if (!dest || prev.find(dest) == prev.end()) {
            std::cout << "⚠️ No se encontró camino (prev no contiene a destino).\n";
            return;
        }

        Node* current = dest;

        while (current && prev.find(current) != prev.end() && prev.at(current) != nullptr) {
            Node* from = prev.at(current);
            sf::VertexArray line(sf::Lines, 2);
            line[0].position = from->coord;
            line[1].position = current->coord;
            line[0].color = sf::Color::Magenta;
            line[1].color = sf::Color::Magenta;
            path.push_back(line);
            current = from;
        }

        if (path.empty()) {
            std::cout << "⚠️ No se encontró camino.\n";
        } else {
            std::cout << "✅ Camino construido con " << path.size() << " segmentos.\n";
        }
    }

    void dijkstra(Graph& graph) {
        std::priority_queue<std::pair<float, Node*>, std::vector<std::pair<float, Node*>>, std::greater<>> pq;
        std::unordered_map<Node*, float> dist;
        std::unordered_map<Node*, Node*> prev;

        for (auto& pair : graph.nodes) {
            dist[pair.second] = std::numeric_limits<float>::infinity();
            prev[pair.second] = nullptr;
        }

        dist[src] = 0.0f;
        pq.push({0.0f, src});

        while (!pq.empty()) {
            auto [cost, u] = pq.top();
            pq.pop();

            for (Edge* edge : u->edges) {
                Node* v = edge->dest;
                float alt = dist[u] + edge->length;
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.push({alt, v});
                    draw_edge(u, v, sf::Color::Yellow);
                }
            }
        }

        set_final_path(prev);
    }

    void greedy(Graph& graph) {
        auto cmp = [&](Node* a, Node* b) {
            return heuristic(a, dest) > heuristic(b, dest);
        };

        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
        std::unordered_map<Node*, Node*> prev;
        std::unordered_map<Node*, bool> visited;

        open.push(src);
        visited[src] = true;

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();

            if (current == dest) break;

            for (Edge* edge : current->edges) {
                Node* neighbor = edge->dest;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    prev[neighbor] = current;
                    open.push(neighbor);
                    draw_edge(current, neighbor, sf::Color::Cyan);
                }
            }
        }

        set_final_path(prev);
    }

    void a_star(Graph& graph) {
        auto cmp = [](const std::pair<float, Node*>& a, const std::pair<float, Node*>& b) {
            return a.first > b.first;
        };

        std::priority_queue<std::pair<float, Node*>, std::vector<std::pair<float, Node*>>, decltype(cmp)> open(cmp);
        std::unordered_map<Node*, float> g_score;
        std::unordered_map<Node*, float> f_score;
        std::unordered_map<Node*, Node*> prev;

        for (auto& [_, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<float>::infinity();
            f_score[node] = std::numeric_limits<float>::infinity();
        }

        if (!src || !dest) return;

        g_score[src] = 0.0f;
        f_score[src] = heuristic(src, dest);
        open.push({f_score[src], src});
        prev[src] = nullptr;

        while (!open.empty()) {
            Node* current = open.top().second;
            open.pop();

            if (!current) continue;
            if (current == dest) break;

            for (Edge* edge : current->edges) {
                if (!edge || !edge->dest) continue;

                Node* neighbor = edge->dest;
                float tentative_g = g_score[current] + edge->length;

                if (tentative_g < g_score[neighbor]) {
                    prev[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    f_score[neighbor] = tentative_g + heuristic(neighbor, dest);
                    open.push({f_score[neighbor], neighbor});
                    draw_edge(current, neighbor, sf::Color::Blue);
                }
            }
        }

        if (prev.find(dest) != prev.end()) {
            set_final_path(prev);
        } else {
            std::cout << "⚠️ No se encontró camino (destino inalcanzable).\n";
        }
    }
};

#endif
