from grader import Grader


def test_a():
    n_queens_solver = submission.BacktrackingSearch()
    n_queens_solver.solve(submission.create_n_queens_csp(8))
    grader.require_is_equal(92, n_queens_solver.num_assignments)
    grader.require_is_equal(2057, n_queens_solver.num_operations)


def test_b():
    mcv_solver = submission.BacktrackingSearch()
    mcv_solver.solve(submission.create_n_queens_csp(8), mcv=True)
    grader.require_is_equal(92, mcv_solver.num_assignments)
    grader.require_is_equal(1361, mcv_solver.num_operations)


def test_c():
    ac_solver = submission.BacktrackingSearch()
    ac_solver.solve(submission.create_n_queens_csp(8), ac3=True)
    grader.require_is_equal(92, ac_solver.num_assignments)
    grader.require_is_equal(21, ac_solver.first_assignment_num_operations)
    grader.require_is_equal(769, ac_solver.num_operations)


grader = Grader()
submission = grader.load('submission')
grader.add_part('a', test_a, 5, description='Test for Create 8-Queens CSP')
grader.add_part('b', test_b, 5, description='Test for MCV with 8-Queens CSP')
grader.add_part('c', test_c, 10, description='Test for AC-3 with n-queens CSP')
grader.grade()
