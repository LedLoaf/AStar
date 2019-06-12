#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <utility>
//! LEFT OFF 32:32
using namespace std;
sf::Vector2u WINDOW_SIZE{800,480};

/* Background
~~~~~~~~~~
The A* path finding algorithm is a widely used and powerful shortest path
finding node traversal algorithm. A heuristic is used to bias the algorithm
towards success. This code is probably more interesting than the video. :-/*/
struct AStar {
public:
	AStar(sf::RenderWindow* window) : window{window} {

		this->shape.setSize(sf::Vector2f{(float)this->nNodeSize - this->nNodeBorder,(float)this->nNodeSize - this->nNodeBorder});

		// Create a 2D array of nodes - this is for convenience of rendering and construction
		// and is not required for the algorithm to work - the nodes could be placed anywhere
		// in any space, in multiple dimensions...
		nodes = new sNode[nMapWidth * nMapHeight];

		for (auto x = 0; x < nMapWidth; x++) {
			for (auto y = 0; y < nMapHeight; y++) {
				nodes[y * nMapWidth + x].x = x;		// ...because we give each node its own coordinates
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;

			}
		}

		// Create connections - in this case nodes are on a regular grid
		for (auto x = 0; x < nMapWidth; x++) {
			for (auto y = 0; y < nMapHeight; y++) {
				if (y > 0)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if (y < nMapHeight - 1)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x > 0)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if (x < nMapWidth - 1)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);

				// if diagnals are included
				if (b8Connection) {
					// We can also connect diagonally
					if (y > 0 && x > 0)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x - 1)]);
					if (y < nMapHeight - 1 && x>0)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x - 1)]);
					if (y > 0 && x < nMapWidth - 1)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x + 1)]);
					if (y < nMapHeight - 1 && x < nMapWidth - 1)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x + 1)]);
				}
			}
		}
		// Manually position the start and end markers so they are not nullptr
		nodeStart = &nodes[(nMapHeight / 2) * nMapWidth + 1];
		nodeEnd = &nodes[(nMapHeight / 2) * nMapWidth + nMapWidth - 2];



		this->font.loadFromFile("assets/fonts/sansation.ttf");
		this->text.setFont(this->font);
		this->text.setCharacterSize(12);

	}

	void toggleDiagnols() {

		for (auto x = 0; x < nMapWidth; x++) {
			for (auto y = 0; y < nMapHeight; y++) {
				nodes[y * nMapWidth + x].x = x;		// ...because we give each node its own coordinates
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = nodes[y * nMapWidth + x].bObstacle;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;
				nodes[y * nMapWidth + x].vecNeighbors.clear();

			}
		}
		// Create connections - in this case nodes are on a regular grid
		for (auto x = 0; x < nMapWidth; x++) {
			for (auto y = 0; y < nMapHeight; y++) {
				if (y > 0)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if (y < nMapHeight - 1)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x > 0)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if (x < nMapWidth - 1)
					nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);

				// if diagnals are included
				if (b8Connection) {
					// We can also connect diagonally
					if (y > 0 && x > 0)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x - 1)]);
					if (y < nMapHeight - 1 && x>0)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x - 1)]);
					if (y > 0 && x < nMapWidth - 1)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y - 1) * nMapWidth + (x + 1)]);
					if (y < nMapHeight - 1 && x < nMapWidth - 1)
						nodes[y * nMapWidth + x].vecNeighbors.push_back(&nodes[(y + 1) * nMapWidth + (x + 1)]);
				}
			}
		}
		SolveAStar();
	}

	bool SolveAStar() {
		// reset navigation graph - default all node states
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				nodes[y * nMapWidth + x].bVisited = false;
				nodes[y * nMapWidth + x].fGlobalGoal = INFINITY;
				nodes[y * nMapWidth + x].fLocalGoal = INFINITY;
				nodes[y * nMapWidth + x].parent = nullptr;			// No Parents

				if (&nodes[y * nMapWidth + x] == nodeStart) {
					shape.setPosition(x * this->nNodeSize + (float)this->nNodeBorder, y * this->nNodeSize + (float)this->nNodeBorder);
					shape.setFillColor(sf::Color{127,255,0,255});
				}

				if (&nodes[y * nMapWidth + x] == nodeEnd) {
					shape.setPosition(x * this->nNodeSize + (float)this->nNodeBorder, y * this->nNodeSize + (float)this->nNodeBorder);
					shape.setFillColor(sf::Color{220,20,60,255});
				}
			}
		}

		auto distance = [](sNode* a, sNode* b)	// for convenience
		{
			return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
		};

		auto heuristic = [distance](sNode* a, sNode* b)		// so we can experiment with heuristic 
		{
			return distance(a, b);
		};

		// setup starting conditions
		sNode* nodeCurrent = nodeStart;
		nodeStart->fLocalGoal = 0.f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

		// Add start node to not tested list - this will ensure it gets tested.
		// As the algorithm progress, newly discovered nodes get added to this list, and will themselves be tested later
		list<sNode*> listNotTestedNodes;
		listNotTestedNodes.push_back(nodeStart);

		// If the not tested list contains nodes, there may be better paths which not yet been explored.
		// However, we will also stop searching when we reach the target - there may well be better paths but this one will do - it won't be the longest
		while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)	// find absolutely shorest path // && nodeCurrent != nodeEnd)
		{
			// Sort untested nodes by global goal, so lowest it first
			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs) { return lhs->fGlobalGoal < rhs->fGlobalGoal; });

			// Front of listedNotTestedNodes is potentially the lowest distance node.
			// Our list may also contain nodes that have been visited, so ditch these...
			while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

			// ...or abort because there are no valid nodes left to test
			if (listNotTestedNodes.empty())
				break;

			nodeCurrent = listNotTestedNodes.front();
			nodeCurrent->bVisited = true;	// We only explore a node once

			// Check each of this node's neighbors...
			for (auto nodeNeighbor : nodeCurrent->vecNeighbors) {
				// ... and only if the neighbor is not visted and is not an obstacle, add it to NotTested List
				if (!nodeNeighbor->bVisited && nodeNeighbor->bObstacle == 0)
				{
					listNotTestedNodes.push_back(nodeNeighbor);
				}

				// Calculate the neighbors potential lowest parent distance
				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbor);

				// If choosing to path through this node is a lower distance than what the neighbor currently has set, update the neighbor to use this node
				// as the path source, and set its distance scores as necessary
				if (fPossiblyLowerGoal < nodeNeighbor->fLocalGoal) {
					nodeNeighbor->parent = nodeCurrent;
					nodeNeighbor->fLocalGoal = fPossiblyLowerGoal;

					// The best path length to the neighbor being tested has changed, so update the neighbor's score.
					// The heuristic is used to globally bias the path algorithm, so its knows if it's getter better or worse.
					// At some point the algorithm will realize this path is worse and abandon it, and then we go and search along the next best path.
					nodeNeighbor->fGlobalGoal = nodeNeighbor->fLocalGoal + heuristic(nodeNeighbor, nodeEnd);
				}


			}
		}
		return true;
	}

	float d = 0.f;

	void onUpdate(float dt, sf::Vector2i mousePos) {
		d += dt;
		this->mousePos = mousePos;
		// Use integer division to nicely get cursor position in node space
		int nSelectedNodeX = mousePos.x / nNodeSize;
		int nSelectedNodeY = mousePos.y / nNodeSize;

		if (d >= 0.355)
		{
			// use mouse to draw maze, shift and ctrl to place start and end
			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
				if (nSelectedNodeX >= 0 && nSelectedNodeX < nMapWidth) {
					if (nSelectedNodeY >= 0 && nSelectedNodeY < nMapHeight) {
						if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
							nodeStart = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
						else if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))
							nodeEnd = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
						else
							nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle = !nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle;

						SolveAStar();	// Solve in "real-time" gives a nice effect.
					}
				}

				d = 0;
			}
		}

		// surround startNode with blocks
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
			auto x = nodeStart->x;
			auto y = nodeStart->y;

			nodes[y * nMapWidth + (x - 1)].bObstacle = true;
			nodes[y * nMapWidth + (x + 1)].bObstacle = true;
			nodes[(y - 1) * nMapWidth + x].bObstacle = true;
			nodes[(y + 1) * nMapWidth + x].bObstacle = true;

			nodes[(y - 1) * nMapWidth + (x - 1)].bObstacle = true;
			nodes[(y - 1) * nMapWidth + (x + 1)].bObstacle = true;
			nodes[(y + 1) * nMapWidth + (x - 1)].bObstacle = true;
			nodes[(y + 1) * nMapWidth + (x + 1)].bObstacle = true;
		}

		// toggles diamond around starting noade
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Enter)) {
			auto x = nodeStart->x;
			auto y = nodeStart->y;

			nodes[(y - 2) * nMapWidth + (x - 2)].bObstacle = true;
			nodes[(y - 2) * nMapWidth + (x + 2)].bObstacle = true;
			nodes[(y + 2) * nMapWidth + (x - 2)].bObstacle = true;
			nodes[(y + 2) * nMapWidth + (x + 2)].bObstacle = true;

			nodes[(y - 2) * nMapWidth + (x - 1)].bObstacle = true;
			nodes[(y - 2) * nMapWidth + (x + 1)].bObstacle = true;
			nodes[(y + 2) * nMapWidth + (x - 1)].bObstacle = true;
			nodes[(y + 2) * nMapWidth + (x + 1)].bObstacle = true;

			nodes[(y - 1) * nMapWidth + (x - 2)].bObstacle = true;
			nodes[(y - 1) * nMapWidth + (x + 2)].bObstacle = true;
			nodes[(y + 1) * nMapWidth + (x - 2)].bObstacle = true;
			nodes[(y + 1) * nMapWidth + (x + 2)].bObstacle = true;

		}


		// toggles text on blocks
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num8)) {
			b8Connection = !b8Connection;
			toggleDiagnols();
			this->window->setTitle("[8]Connectivity: " + std::to_string(b8Connection));
		}

		// sets outline in obsticles
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
			for(int x = 0; x < nMapWidth; x++) {
				for(int y = 0; y < nMapHeight; y++) {
					if (x == 0 || x == nMapWidth-1   || y == 0 || y == nMapHeight - 1)
						nodes[y * nMapWidth + x].bObstacle = true;
				}
			}
		}

		// clear all obstacles
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::C)) {
			for(int x = 0; x < nMapWidth; x++) {
				for(int y = 0; y < nMapHeight; y++) {
						nodes[y * nMapWidth + x].bObstacle = false;
				}
			}
		}

		// Draw Connection First - lines from this nodes position to its connected neighbor node positions
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				for (auto n : nodes[y * nMapWidth + x].vecNeighbors) {
					sf::Vertex line[] = {
						sf::Vertex(sf::Vector2f(x * this->nNodeSize + this->nNodeSize / 2 + 3,
														  y * this->nNodeSize + this->nNodeSize / 2 + 3)),
						sf::Vertex(sf::Vector2f(n->x * this->nNodeSize + this->nNodeSize / 2 + 3,
														  n->y * this->nNodeSize + this->nNodeSize / 2 + 3))
					};


					line->color = sf::Color{30,144,255,255};
					this->window->draw(line, 2, sf::Lines);
				}
			}
		}


		// Draw Nodes on top
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {

				if (nodes[y * nMapWidth + x].bObstacle)
					shape.setFillColor(sf::Color{47,79,79,255});
				else
					shape.setFillColor(sf::Color{0,191,255,255});
				//	shape.setFillColor(sf::Color{0,191,255});

				shape.setPosition(x * this->nNodeSize + this->nNodeBorder, y * this->nNodeSize + this->nNodeBorder);


				if (nodes[y * nMapWidth + x].bVisited) {
					shape.setPosition(x * this->nNodeSize + (float)this->nNodeBorder, y * this->nNodeSize + (float)this->nNodeBorder);
					shape.setFillColor(sf::Color{30,144,255,255});
					//shape.setFillColor(sf::Color{0,191,255});
				}

				if (&nodes[y * nMapWidth + x] == nodeStart) {
					shape.setPosition(x * this->nNodeSize + (float)this->nNodeBorder, y * this->nNodeSize + (float)this->nNodeBorder);
					shape.setFillColor(sf::Color{127,255,0,255});
				}

				if (&nodes[y * nMapWidth + x] == nodeEnd) {
					shape.setPosition(x * this->nNodeSize + (float)this->nNodeBorder, y * this->nNodeSize + (float)this->nNodeBorder);
					shape.setFillColor(sf::Color{220,20,60,255});
				}

				this->window->draw(shape);
			}


			// Draw Path by starting ath the end, and following the parent node trail
			// back to the start - the start node will not have a parent path to follow
			if (nodeEnd != nullptr) {
				sNode* p = nodeEnd;
				while (p->parent != nullptr) {
					sf::Vertex line[]
					{
						sf::Vertex(sf::Vector2f(p->x * this->nNodeSize + this->nNodeSize / 2 + 3, p->y * this->nNodeSize + this->nNodeSize / 2 + 3)),
						sf::Vertex(sf::Vector2f(p->parent->x * this->nNodeSize + this->nNodeSize / 2 + 3, p->parent->y * this->nNodeSize + this->nNodeSize / 2 + 3))
					};
					line->color = sf::Color{255,218,185,255};

					this->window->draw(line, 2, sf::Lines);

					// Set next node to this node's parent
					p = p->parent;
				}
			}

		}
	}

private:
	struct sNode {
		bool bObstacle = false;		// is the node an obstruction
		bool bVisited = false;		// have we searched this node before?
		float fGlobalGoal;			// Distance to goal so far
		float fLocalGoal;			// Distance to goal if we took the alternative route 
		int x;						// Node position in 2D space
		int y;
		std::vector<sNode*> vecNeighbors;		// Connection to neighbors
		sNode* parent;							// Node connecting to this node that offers shortest parent
	};

	const int nMapWidth = WINDOW_SIZE.x / 32;
	const int nMapHeight = WINDOW_SIZE.y / 32;

	const int nNodeSize = 32;
	const int nNodeBorder = 9;
	sNode* nodes = nullptr;

	sNode* nodeStart = nullptr;
	sNode* nodeEnd = nullptr;

	sf::Vector2i mousePos;


	sf::RenderWindow* window = nullptr;

	bool b8Connection = false;

	sf::RectangleShape shape;

	sf::Font font;
	sf::Text text;
};

int main() {
	sf::RenderWindow window{sf::VideoMode{WINDOW_SIZE.x,WINDOW_SIZE.y},"SFML Sandbox"};
	window.setFramerateLimit(10);
	window.setPosition(sf::Vector2i{window.getPosition().x,0});
	sf::Clock dtClock;

	AStar astar(&window);

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed || event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
				window.close();
			if (event.type == sf::Event::KeyPressed) {
				switch (event.key.code) {
					case sf::Keyboard::Enter: cout << "Enter Pressed\n"; break;
					case sf::Keyboard::Space: cout << "Space Pressed\n"; break;
					default: break;
				}
			}
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num1)) {

		}

		window.clear();

			astar.onUpdate(dtClock.restart().asSeconds(), sf::Mouse::getPosition(window));


		window.display();

	}
	return 0;
}