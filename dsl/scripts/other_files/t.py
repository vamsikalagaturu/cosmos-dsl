def construct_abc(alpha_in):
    """
    Construct the alpha beta matrices.
    """
    alpha = []
    beta = []
    nc = 0

    # if alpha is not a list of lists
    if isinstance(alpha_in[0], list):
        for a in alpha_in:
            # if a is not in alpha, add it
            if a not in alpha:
                alpha.append(a)
    elif isinstance(alpha_in[0], float):
        alpha.append(alpha_in)

    # sort alpha
    alpha.sort(reverse=True)

    nc = len(alpha)

    beta = [0.0 for i in range(nc)]

    # construct beta from alpha based on apperance of non-zero elements
    for i, a in enumerate(alpha):
        if a[2] != 0:
            beta[i] = 9.81

    return alpha, beta, nc

alpha = [
         [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
         [0.7, 0.0, 0.7, 0.0, 0.0, 0.0],
         ]

alpha, beta, nc = construct_abc(alpha)

print(alpha)
print()
print(beta)
print()
print(nc)