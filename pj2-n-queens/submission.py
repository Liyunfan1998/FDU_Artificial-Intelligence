import copy
from csp import CSP


def create_n_queens_csp(n=8):
    """Create an N-Queen problem on the board of size n * n.

    You should call csp.add_variable() and csp.add_binary_factor().

    Args:
        n: int, number of queens, or the size of one dimension of the board.

    Returns
        csp: A CSP problem with correctly configured factor tables
        such that it can be solved by a weighted CSP solver
    """
    csp = CSP()
    # TODO: Problem a
    # TODO: BEGIN_YOUR_CODE
    vars = []
    for i in range(1, n + 1):
        # we suppose that on a board, we put in queens with order.
        # In other words, Xi is fixed for each queen,
        # we only have to consider Yi for each queen
        varName = 'Y' + str(i)
        csp.add_variable(varName, range(1, n + 1))
        vars.append(varName)

    rule1 = lambda x, y: x != y
    for var in vars:
        for anotherVar in vars:
            if var != anotherVar:
                # csp.add_binary_factor(var, anotherVar, lambda y1, y2: y1 != y2)
                dist = abs(vars.index(anotherVar) - vars.index(var))
                csp.add_binary_factor(var, anotherVar, lambda y1, y2: y1 != y2 and abs(y1 - y2) != dist)

    # raise NotImplementedError
    # TODO: END_YOUR_CODE
    return csp


class BacktrackingSearch:
    """A backtracking algorithm that solves CSP.

    Attributes:
        num_assignments: keep track of the number of assignments
            (identical when the CSP is unweighted)
        num_operations: keep track of number of times backtrack() gets called
        first_assignment_num_operations: keep track of number of operations to
            get to the very first successful assignment (maybe not optimal)
        all_assignments: list of all solutions found

        csp: a weighted CSP to be solved
        mcv: bool, if True, use Most Constrained Variable heuristics
        ac3: bool, if True, AC-3 will be used after each variable is made
        domains: dictionary of domains of every variable in the CSP

    Usage:
        search = BacktrackingSearch()
        search.solve(csp)
    """

    def __init__(self):
        self.num_assignments = 0
        self.num_operations = 0
        self.first_assignment_num_operations = 0
        self.all_assignments = []

        self.csp = None
        self.mcv = False
        self.ac3 = False
        self.domains = {}

    def reset_results(self):
        """Resets the statistics of the different aspects of the CSP solver."""
        self.num_assignments = 0
        self.num_operations = 0
        self.first_assignment_num_operations = 0
        self.all_assignments = []

    def check_factors(self, assignment, var, val):
        """Check consistency between current assignment and a new variable.

        Given a CSP, a partial assignment, and a proposed new value for a
        variable, return the change of weights after assigning the variable
        with the proposed value.

        Args:
            assignment: A dictionary of current assignment.
                Unassigned variables do not have entries, while an assigned
                variable has the assigned value as value in dictionary.
                e.g. if the domain of the variable A is [5,6],
                and 6 was assigned to it, then assignment[A] == 6.
            var: name of an unassigned variable.
            val: the proposed value.

        Returns:
            bool
                True if the new variable with value can satisfy constraint,
                otherwise, False
        """
        assert var not in assignment
        if self.csp.unary_factors[var]:
            if self.csp.unary_factors[var][val] == 0:
                return False
        for var2, factor in self.csp.binary_factors[var].items():
            if var2 not in assignment:
                continue
            if factor[val][assignment[var2]] == 0:
                return False
        return True

    def solve(self, csp, mcv=False, ac3=False):
        """Solves the given unweighted CSP using heuristics.

        Note that we want this function to find all possible assignments.
        The results are stored in the variables described in
            reset_result().

        Args:
            csp: A unweighted CSP.
            mcv: bool, if True, Most Constrained Variable heuristics is used.
            ac3: bool, if True, AC-3 will be used after each assignment of an
            variable is made.
        """
        self.csp = csp
        self.mcv = mcv
        self.ac3 = ac3
        self.reset_results()
        self.domains = {var: list(self.csp.values[var])
                        for var in self.csp.variables}
        self.backtrack({})

    def backtrack(self, assignment):
        """Back-tracking algorithms to find all possible solutions to the CSP.

        Args:
            assignment: a dictionary of current assignment.
                Unassigned variables do not have entries, while an assigned
                variable has the assigned value as value in dictionary.
                    e.g. if the domain of the variable A is [5, 6],
                    and 6 was assigned to it, then assignment[A] == 6.
        """
        self.num_operations += 1
        num_assigned = len(assignment.keys())
        if num_assigned == self.csp.vars_num:
            self.num_assignments += 1
            new_assignment = {}
            for var in self.csp.variables:
                new_assignment[var] = assignment[var]
            self.all_assignments.append(new_assignment)
            if self.first_assignment_num_operations == 0:
                self.first_assignment_num_operations = self.num_operations
            return

        var = self.get_unassigned_variable(assignment)
        ordered_values = self.domains[var]

        if not self.ac3:
            for value in ordered_values:
                if self.check_factors(assignment, var, value):
                    assignment[var] = value
                    self.backtrack(assignment)
                    del assignment[var]
        else:
            for value in ordered_values:
                if self.check_factors(assignment, var, value):
                    assignment[var] = value
                    local_copy = copy.deepcopy(self.domains)
                    self.domains[var] = [value]
                    self.arc_consistency_check(var)
                    self.backtrack(assignment)
                    self.domains = local_copy
                    del assignment[var]

    def get_unassigned_variable(self, assignment):
        """Get a currently unassigned variable for a partial assignment.

        If mcv is True, Use heuristic: most constrained variable (MCV)
        Otherwise, select a variable without any heuristics.

        Most Constrained Variable (MCV):
            Select a variable with the least number of remaining domain values.
            Hint: self.domains[var] gives you all the possible values
            Hint: get_delta_weight gives the change in weights given a partial
                assignment, a variable, and a proposed value to this variable
            Hint: choose the variable with lowest index in self.csp.variables
                for ties

        Args:
            assignment: a dictionary of current assignment.

        Returns
            var: a currently unassigned variable.
        """
        if not self.mcv:
            for var in self.csp.variables:
                if var not in assignment:
                    return var
        else:
            # TODO: Problem b
            # TODO: BEGIN_YOUR_CODE
            mcv1 = self.csp.variables[0]
            domain_mcv = float('inf')  # the basic idea is to find min domain
            for var in self.csp.variables:
                if var not in assignment:
                    var_domain = 0
                    for value in self.domains[var]:
                        if self.check_factors(assignment, var, value) != 0:
                            var_domain += 1
                    if var_domain < domain_mcv:  # choose a var with the least domain space
                        domain_mcv = var_domain
                        mcv1 = var
            return mcv1
            # raise NotImplementedError
            # TODO: END_YOUR_CODE

    def arc_consistency_check(self, var):
        """AC-3 algorithm.

        The goal is to reduce the size of the domain values for the unassigned
        variables based on arc consistency.

        Hint: get variables neighboring variable var:
            self.csp.get_neighbor_vars(var)

        Hint: check if a value or two values are inconsistent:
            For unary factors
                self.csp.unaryFactors[var1][val1] == 0
            For binary factors
                self.csp.binaryFactors[var1][var2][val1][val2] == 0

        Args:
            var: the variable whose value has just been set
        """
        # TODO: Problem c
        # TODO: BEGIN_YOUR_CODE
        queue = [(neighbor, var) for neighbor in self.csp.get_neighbor_vars(var)]

        # q = Queue.Queue()
        # for arc in self.csp.binary_factors:
        #     q.put(arc)

        def remove_inconsistent_values(x1, x2):
            removed = False
            """
            tmp1 = self.domains[x1]
            tmp2 = [value for value in self.domains[x1]]
            if sum(tmp1) != sum(tmp2):
                print("stop")
            for x1v in self.domains[x1]:
            """
            unchangedDomains = list(self.domains[x1])

            # for x1v in [value for value in self.domains[x1]]:
            # this is a unchanged version of self.domains[x1]
            # as we change self.domains[x1] in the following for loop,
            # we would like our domain to stay the same in the conditions of the for loop
            for x1v in unchangedDomains:
                consistent = False
                for x2v in self.domains[x2]:
                    if self.csp.binary_factors[x1][x2][x1v][x2v] != 0:
                        consistent = True
                        # break
                if not consistent:
                    self.domains[x1].remove(x1v)
                    removed = True
            return removed

        while len(queue):
            x1, x2 = queue.pop(0)  # pop out the first arc (list type can also do pop!!!)
            if remove_inconsistent_values(x1, x2):
                for xk in self.csp.get_neighbor_vars(x1):
                    if xk != x2:
                        queue.append((xk, x1))

        # raise NotImplementedError
        # TODO: END_YOUR_CODE
