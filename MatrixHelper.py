import math
import numpy as np

DET_THRESHOLD = 0.0000001
K_ADJ = 0.0001  # Will add K_ADJ to each eigenvalue

# Get a safe, adjusted pseudoinverse that will account for singularities. 

def GetAdjustedPInv(matrixIn):
    # Base formula is (A*A^T)^(-1)*A^T
    # Corrected formula is (A*A^T+k*I)^(-1)*A^T to ensure that the lhs term is invertible
    # Source https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
    transpose = np.transpose(matrixIn)
    raw = transpose @ matrixIn

    # Then, check determinant to see if it is invertible. 
    while (math.fabs(np.linalg.det(raw)) < DET_THRESHOLD):
        # Iteratively apply correction
        # Adjust a matrix by adding K_ADJ to each element along the main diagonal
        for i in range(len(raw)):
            raw[i][i] += K_ADJ
    
    # Now, this matrix should be safely invertible

    inv = np.linalg.inv(raw)
    return inv @ transpose