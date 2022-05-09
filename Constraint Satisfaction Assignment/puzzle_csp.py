# Look for #IMPLEMENT tags in this file.
'''
All models need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = caged_csp_model(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the FunPuzz puzzle.

The grid-only models do not need to encode the cage constraints.

1. binary_ne_grid (worth 10/100 marks)
    - A model of a FunPuzz grid (without cage constraints) built using only 
      binary not-equal constraints for both the row and column constraints.

2. nary_ad_grid (worth 10/100 marks)
    - A model of a FunPuzz grid (without cage constraints) built using only n-ary 
      all-different constraints for both the row and column constraints. 

3. caged_csp_model (worth 25/100 marks) 
    - A model built using your choice of (1) binary binary not-equal, or (2) 
      n-ary all-different constraints for the grid.
    - Together with FunPuzz cage constraints.

'''
from cspbase import *
import itertools


def binary_ne_grid(fpuzz_grid):
    ##IMPLEMENT
    constraints, constraint_values, constraint_names = [], [], []
    initial_variables = get_initial_variables(fpuzz_grid)
    size_of_board = initial_variables["size"]
    cell_values = initial_variables["cell_values"]
    variables = initial_variables["variables"]

    for cell in itertools.product(list(range(size_of_board)), list(range(size_of_board)), list(range(size_of_board))):
        col_c1 = cell_name(cell[1], cell[0]) + ", " + cell_name(cell[2], cell[0])
        col_c2 = cell_name(cell[2], cell[0]) + ", " + cell_name(cell[1], cell[0])
        if col_c1 not in constraint_names and col_c2 not in constraint_names and cell[2] != cell[1]:
            satisfying_col_constraints = []
            for v1 in cell_values:
                for v2 in cell_values:
                    if v1 != v2:
                        satisfying_col_constraints.append((v1, v2))
            c1 = Constraint(cell_name(cell[1], cell[0]) + ", " + cell_name(cell[2], cell[0]),
                            [variables[cell[1]][cell[0]], variables[cell[2]][cell[0]]])
            c1.add_satisfying_tuples(satisfying_col_constraints)
            constraints.append(c1)
            constraint_values.append(c1)
            constraint_names.append(c1.name)
            row_c1 = cell_name(cell[0], cell[1]) + ", " + cell_name(cell[0], cell[2])
            row_c2 = cell_name(cell[0], cell[2]) + ", " + cell_name(cell[0], cell[1])
            if row_c1 not in constraint_names and row_c2 not in constraint_names and cell[2] != cell[1]:
                satisfying_row_constraints = []
                for v1 in cell_values:
                    for v2 in cell_values:
                        if v1 != v2:
                            satisfying_row_constraints.append((v1, v2))

                added_constraints = Constraint(cell_name(cell[0], cell[1]) + ", " + cell_name(cell[0], cell[2]),
                                               [variables[cell[0]][cell[1]], variables[cell[0]][cell[2]]])
                added_constraints.add_satisfying_tuples(satisfying_row_constraints)
                constraint_values.append(added_constraints)
                constraint_names.append(added_constraints.name)

    csp = CSP('binary_ne', [variable for rows in variables for variable in rows])
    for constraint in constraints:
        csp.add_constraint(constraint)
    return (csp, variables)


def nary_ad_grid(fpuzz_grid):
    ##IMPLEMENT
    constraints, scope = [], []
    initial_variables = get_initial_variables(fpuzz_grid)
    cell_values = initial_variables["cell_values"]
    variables = initial_variables["variables"]

    for col in cell_values:
        for row in cell_values:
            scope.append(variables[row][col])
        cells1 = []
        for value_pair1 in itertools.permutations(cell_values):
            cells1.append(value_pair1)
        c1 = Constraint(hash(col), scope)
        c1.add_satisfying_tuples(cells1)
        constraints.append(c1)
        cells2 = []
        for value_pair2 in itertools.permutations(cell_values):
            cells2.append(value_pair2)
        c2 = Constraint(hash(col), variables[col])
        c2.add_satisfying_tuples(cells2)
        constraints.append(c2)

    csp = CSP('nary_ad', [variable for rows in variables for variable in rows])
    for constraint in constraints:
        csp.add_constraint(constraint)
    return (csp, variables)


def caged_csp_model(fpuzz_grid):
    ##IMPLEMENT
    constraints, constraint_values, constraint_names = [], [], []
    initial_variables = get_initial_variables(fpuzz_grid)
    size_of_board = initial_variables["size"]
    cell_values = initial_variables["cell_values"]
    cage_constraints = range(1, size_of_board)
    csp, variables = binary_ne_grid(fpuzz_grid)

    for cage in cage_constraints:
        row = list(fpuzz_grid[cage])
        operation, target, scope_values = row[-1], row[-2], row[:-2]
        scope, cells = [], []
        for scope_value in scope_values:
            value = variables[(scope_value // 10) - 1][(scope_value % 10) - 1]
            scope.append(value)
        constraint_name = "Operation: " + str(operation) + "Target:" + str(target)
        constraint = Constraint(constraint_name, scope)
        op = check_operation(operation)
        if op['addition']:
            for cell in itertools.product(tuple(cell_values), repeat=len(scope)):
                if sum(cell) == target:
                    cells.append(cell)
        elif op['subtraction']:
            for cell in itertools.product(tuple(cell_values), repeat=len(scope)):
                for i in range(len(scope)):
                    difference = cell[i] - sum(cell[:i] + cell[i + 1:])
                    if difference == target:
                        cells.append(cell)
        elif op['multiplication']:
            for cell in itertools.product(tuple(cell_values), repeat=len(scope)):
                for i in range(len(scope)):
                    product = float(cell[i])
                    for v1 in cell[:i] + cell[i + 1:]:
                        product *= v1
                    if product == target:
                        cells.append(cell)
        elif op['division']:
            for cell in itertools.product(tuple(cell_values), repeat=len(scope)):
                for i in range(len(scope)):
                    quotient = float(cell[i])
                    for v1 in cell[:i] + cell[i + 1:]:
                        quotient = quotient / v1
                    if quotient == target:
                        cells.append(cell)

        constraint.add_satisfying_tuples(cells)
        constraints.append(constraint)
        constraint_values.append(constraint)
        constraint_names.append(constraint.name)

    for constraint in constraints:
        csp.add_constraint(constraint)
    return (csp, variables)


def check_operation(operation):
    operation_dictionary = {'addition': False, 'subtraction': False, 'multiplication': False, 'division': False}
    if operation == 0:
        operation_dictionary['addition'] = True
    elif operation == 1:
        operation_dictionary['subtraction'] = True
    elif operation == 3:
        operation_dictionary['multiplication'] = True
    elif operation == 2:
        operation_dictionary['division'] = True
    return operation_dictionary


def cell_name(row, column):
    # Return cell name used for constraints
    return "Row: " + str(row) + " Col: " + str(column)


def get_initial_variables(fpuzz_grid):
    # Return size_of_board, cell_values, variables in that order
    size_of_board = fpuzz_grid[0][0]
    max_value = size_of_board + 1
    cell_values = list(range(1, max_value))
    variables = []
    for r in range(size_of_board):
        row = []
        for c in range(size_of_board):
            variable = Variable(cell_name(r, c), list(range(1, size_of_board + 1))[:])
            row.append(variable)
        variables.append(row)
    return {"size": size_of_board, "cell_values": cell_values, "variables": variables}
