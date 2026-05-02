# A* Path Planning Algorithm (Autonomous Mobile Robots)

## 📌 Overview

This project implements the **A* (A-Star) path planning algorithm** for a 2D grid-based environment using Python.

It was developed as part of the *Autonomous Mobile Robots* coursework, where the objective was to compute the most efficient path between a start and goal position while avoiding obstacles.

The system includes a graphical user interface (GUI) that allows users to interactively define the environment and visualize the computed path.

---

## 🎯 Objectives

* Implement the A* search algorithm from first principles
* Apply heuristic-based pathfinding using Euclidean distance
* Enable obstacle-aware navigation in a grid environment
* Visualize path planning in real time using a GUI

---

## ⚙️ Features

* Interactive grid-based environment
* Add/remove obstacles dynamically
* Define start and goal positions
* Real-time path visualization
* Error handling and debug messages
* Efficient pathfinding using heuristic search

---

## 🧠 Algorithm Explanation

The A* algorithm evaluates nodes using:

* **g(n)** → Cost from start to current node
* **h(n)** → Heuristic (Euclidean distance to goal)
* **f(n) = g(n) + h(n)**

It always expands the node with the lowest **f(n)** value, ensuring an optimal path when the heuristic is admissible.

This implementation follows the original formulation described in the foundational paper .

---

## 📂 Project Structure

```id="1g7cxg"
A-star-algorithm-Path-Planning/
│
├── src/
│   ├── gui.py                     # GUI application (DO NOT MODIFY)
│   └── A_star_path_planner.py     # A* algorithm implementation
│
├── test run/
│   ├── test screenshot.png         
│   └── A star Path Planner.mp4   
│
└── README.md
```

---

## ▶️ How to Run

### 1. Clone the repository

```id="x1gqru"
git clone https://github.com/anspermiranda/A-star-algorithm-Path-Planning.git
cd A-star-algorithm-Path-Planning
```

---

### 2. Install dependencies

```id="zqqn8x"
pip install PyQt5
```

---

### 3. Run the application

```id="8y7z4c"
python src/gui.py
```

---

## 🖥️ How to Use the GUI

According to the interface (see coursework description, page 5 ):

1. Set grid size (Width & Height)
2. Click **Add Obstacles** → draw walls (black cells)
3. Click **Add Start** → select start point (green)
4. Click **Add End** → select goal point (red)
5. Click **Run**

### Output:

* 🔵 Blue → computed path
* ⚫ Black → obstacles
* 🟢 Green → start
* 🔴 Red → goal

---

## 🎥 Test Run


![A* Path Planning Demo](test%20run/Screenshot%20test.png)


---

## 📊 Example Output

The algorithm successfully finds the shortest path while avoiding obstacles and dynamically updates the GUI.

---

## 📚 Learning Outcomes

This project demonstrates:

* Heuristic search algorithms (A*)
* Graph traversal and optimization
* Real-time visualization using GUI frameworks
* Handling constraints (grid boundaries, obstacles)
* Efficient data structures (dictionary-based frontier)

---

## 🚀 Future Improvements

* Add diagonal movement
* Implement Dijkstra and compare performance
* Add weighted grids
* Visualize explored nodes
* Convert to ROS-based planner

---

## 🏫 Academic Context

Developed for:
**AERO60492 – Autonomous Mobile Robots**

The coursework required implementing the A* algorithm inside a provided GUI framework without using external libraries.

---

## 👤 Author

Ansper Miranda
