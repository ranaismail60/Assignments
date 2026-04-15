# 🧩 Sudoku Solver using CSP

This project implements a **Sudoku Solver** using **Constraint Satisfaction Problem (CSP)** techniques.

---

##  Features
-  Backtracking Search
-  AC-3 (Arc Consistency)
-  Forward Checking
-  MRV Heuristic (Minimum Remaining Values)

---

##  Problem Definition
Solve a 9×9 Sudoku grid such that:
- Each row has digits 1–9
- Each column has digits 1–9
- Each 3×3 box has digits 1–9

---

##  Approach

### 1. CSP Formulation
- Variables → Each cell `(row, col)`
- Domain → `{1–9}` or fixed value
- Constraints → Row, Column, Box

---

### 2. Algorithms Used

#### 🔹 AC-3
- Enforces arc consistency
- Reduces domain before search

#### 🔹 Backtracking
- DFS-based search

#### 🔹 Forward Checking
- Removes invalid values from neighbors

#### 🔹 MRV Heuristic
- Picks variable with smallest domain

---

##  Input Format
- 9 lines of digits
- `0` represents empty cell

Example:
