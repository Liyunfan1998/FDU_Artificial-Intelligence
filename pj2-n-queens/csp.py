"""A Weighted Constraint Satisfied Problem.

All variables are referenced by their index instead of their original names.

To solve N-Queens problem, only unweighted CSP is needed, which means
the factor values of the variables are either 0 or 1.

Author: GAO Yixu
    Reference: Stanford CS221 codes for Course Scheduling
    
Usage:
    # Create a CSP
    csp = CSP()
    # Add variables
    csp.add_variable('A', [1, 2])
    csp.add_variable('B', [2, 3])
    # Add unary factors
    csp.add_unary_factor('B', lambda y: 1.0 / y)
    # to get unary factor value given a variable its value
    f = csp.unary_factors[variable][value]
    # Add binary factors
    csp.add_binary_factor('A', 'B', lambda x, y: x != y)
    # to get binary factor value
    # given a variable1 with value1 variable2 with value2
    f = csp.binary_factors[variable1][variable2][value1][value2]

Examples:
    1. Create a weighted CSP.
    csp = CSP()
    csp.add_variable('A', [1, 2, 3])
    csp.add_variable('B', [1, 2, 3, 4, 5])
    csp.add_unary_factor('A', lambda x: x > 1)
    csp.add_unary_factor('A', lambda x: x != 2)
    csp.add_unary_factor('B', lambda y: 1.0 / y)
    csp.add_binary_factor('A', 'B', lambda x, y: x != y)

    2. Create a classic CSP of coloring the map of Australia with 3 colors.
    csp = CSP()
    provinces = ['WA', 'NT', 'Q', 'NSW', 'V', 'SA', 'T']
    neighbors = {
        'SA': ['WA', 'NT', 'Q', 'NSW', 'V'],
        'NT': ['WA', 'Q'],
        'NSW': ['Q', 'V']
    }
    colors = ['red', 'blue', 'green']
    def are_neighbors(a, b):
        return (a in neighbors and b in neighbors[a]) or \
               (b in neighbors and a in neighbors[b])
    # Add the variables and binary factors
    for p in provinces:
        csp.add_variable(p, colors)
    for p1 in provinces:
        for p2 in provinces:
            if are_neighbors(p1, p2):
                # Neighbors cannot have the same color
                csp.add_binary_factor(p1, p2, lambda x, y: x != y)
"""


class CSP:
    """A Constraint Satisfied Problem.

    Attributes:
        vars_num: int, total number of variables
        variables: list, variable names, which can be any hashable objects,
            e.g. int, str, or any tuple with hashable objects
        values: dictionary,
            {<variable (K) name>: list of domain values that K can take on}
            e.g. {'A': [1, 2, 3]}
        unary_factors: dictionary, variable weight distribution
            the key is variable name and the value is another dictionary
                the value dictionary contains the weight (float) of the value
                    {<variable name>: {<value>: <float>}}
            e.g. if B in ['a', 'b'] is a variable, and we added two unary
            factor functions f1, f2 for B, then
                unary_factors[B]['a'] == f1('a') * f2('a')
        binary_factors: dictionary of dictionary,
            {<variable1>:
                {<variable2>:
                    {<value1>:
                        {<value2>: <float>}}
            e.g. suppose we have two variables:
                A in ['b', 'c'], B in ['a', 'b']
            we've added two binary functions f1(A,B), f2(A,B) to the CSP, then
                binary_factors[A][B]['b']['a'] == f1('b','a') * f2('b','a')
            binary_factors[A][A] should return a key error
                (since a variable shouldn't have a binary factor with itself)
    """

    def __init__(self):
        self.vars_num = 0
        self.variables = []
        self.values = {}
        self.unary_factors = {}
        self.binary_factors = {}

    def add_variable(self, var, domain):
        """Add a new variable to the CSP.

        Args:
            var: variable name
            domain: list of domain values that K can take on
        """
        if var in self.variables:
            raise Exception("Variable name already exists: %s" % str(var))
        self.vars_num += 1
        self.variables.append(var)
        self.values[var] = domain
        self.unary_factors[var] = None
        self.binary_factors[var] = {}

    def get_neighbor_vars(self, var):
        """Get neighbor variable names of the given variable.

        Args:
            var: variable name

        Returns:
            a list of variables which are neighbors of |var|
        """
        return self.binary_factors[var].keys()

    def add_unary_factor(self, var, factor_function):
        """Add a unary factor function for a variable.

        Its factor value across the domain will be merged with any previously
        added unary factor functions through element-wise multiplication.

        Args:
            var: variable name
            factor_function: a function name
                that can be applied to all values of the variable
        """
        factor = {value: float(factor_function(value))
                  for value in self.values[var]}
        if self.unary_factors[var] is not None:
            assert len(self.unary_factors[var]) == len(factor)
            self.unary_factors[var] = {
                value: self.unary_factors[var][value] * factor[value]
                for value in factor}
        else:
            self.unary_factors[var] = factor

    def add_binary_factor(self, var1, var2, factor_function):
        """Add 2 variable names and a binary factor function to binary_factors.

        If the two variables already had binaryFactors added earlier,
        they will be merged through element-wise multiplication.

        Args:
            var1: variable1 name
            var2: variable2 name
            factor_function: a function name
                that can be applied to all values of the variable
                i.e. factor_function(value1, value2)
        """
        if var1 == var2:
            raise Exception(
                'You are adding a binary factor over a same variable!')
        self._update_binary_factor_table(
            var1, var2, {val1: {val2: float(factor_function(val1, val2))
                                for val2 in self.values[var2]}
                         for val1 in self.values[var1]})
        self._update_binary_factor_table(
            var2, var1, {val2: {val1: float(factor_function(val1, val2))
                                for val1 in self.values[var1]}
                         for val2 in self.values[var2]})

    def _update_binary_factor_table(self, var1, var2, table):
        """Update the binary factor table for binary_factors[var1][var2].

        If it exists, element-wise multiplications will be performed to merge
        them together.

        Args:
            var1: variable1 name
            var2: variable2 name
            table: dictionary of dictionary,
                {value1: {value2: <float>}}
        """
        if var2 not in self.binary_factors[var1]:
            self.binary_factors[var1][var2] = table
        else:
            current_table = self.binary_factors[var1][var2]
            for i in table:
                for j in table[i]:
                    assert i in current_table and j in current_table[i]
                    current_table[i][j] *= table[i][j]
