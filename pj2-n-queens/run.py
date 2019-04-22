import submission


def main():
    csp = submission.create_n_queens_csp()
    alg = submission.BacktrackingSearch()
    alg.solve(csp, mcv=True, ac3=True)
    print('One of the optimal assignments:', alg.all_assignments[0])


if __name__ == '__main__':
    main()
