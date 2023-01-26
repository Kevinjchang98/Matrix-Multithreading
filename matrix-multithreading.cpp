#include <iostream>
#include <thread>
#include <vector>

constexpr int NUM_THREADS = 10;
constexpr int SIZE = 40000;
constexpr bool PRINT_MATRICES = false;

/**
 * Vector object to be used for matrix-vector multiplication
 */
class Vector {
 private:
  double *vector;

 public:
  /**
   * Creates a vector of specified size. If randomize is true, random values will be used, if not, vectors will be
   * initialized with 2
   *
   * @param size Number of elements
   * @param randomize If elements should be randomized
   */
  Vector(int size, bool randomize) {
    this->vector = new double[size];

    for (int i = 0; i < size; i++)
      vector[i] = randomize ? (rand() / 10000) : 2;

  }

  ~Vector() {
    delete[] vector;
  }

  double &operator[](int index) {
    return vector[index];
  }
};

class Matrix {
 private:
  double *matrix;
  int size;

 public:
  /**
   * 2D matrix of numbers. If randomize is true, random numbers will be used, if not, matrix will be initialized with 1
   * @param size Number of elements
   * @param randomize If elements should be randomized
   */
  Matrix(int size, bool randomize) {
    this->size = size;

    this->matrix = new double[size * size];

    for (int i = 0; i < size * size; i++)
      matrix[i] = randomize ? (rand() / 10000) : 1;

  }

  ~Matrix() {
    delete[] matrix;
  }

  /**
   * Can use overloaded () operator to get a value at specific row or column. Does not guard against invalid row or col
   * @param row Row to search
   * @param col Column to search
   * @return Element requested
   */
  double &operator()(int row, int col) {
    return matrix[row * size + col];
  }
};

/**
 * Performs matrix-vector multiplication. Puts answers into ans Matrix. Only performs multiplication on specified
 * rowStart and for the specified numRows
 *
 * @param matrix Matrix to use for multiplication
 * @param vector Vector to use for multiplication
 * @param ans Matrix to put answers into
 * @param rowStart Row to start on; inclusive
 * @param numRows Number of rows including rowStart to use
 */
void matrixVectorMultiplication(std::shared_ptr<Matrix> matrix,
                                std::shared_ptr<Vector> vector,
                                std::shared_ptr<Matrix> ans,
                                int rowStart,
                                int numRows) {
  for (int row = rowStart; row < rowStart + numRows; row++) {
    for (int i = 0; i < SIZE; i++) {
      (*ans)(row, i) = (*matrix)(row, i) * (*vector)[i];
    }
  }
}

/**
 * Prints all values of a matrix
 * @param matrix Source to get values from
 * @param header String to print before values of matrix
 */
void printMatrix(const std::shared_ptr<Matrix> &matrix, const std::string &header) {
  if (header.length() > 0) std::cout << header << "\n";

  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++)
      std::cout << (*matrix)(i, j) << " ";
    std::cout << "\n";
  }

  std::cout << std::endl;
}

/**
 * Multiplies a 2D matrix with a 1D array together and writes answer to answer matrix. Does not modify source matrix and
 * vector
 *
 * @param matrix Matrix to multiply
 * @param vector Vector to multiply
 * @param ans Answer of type Matrix to put answers into. Must be same dimensions as matrix
 */
void runCalculations(std::shared_ptr<Matrix> matrix, std::shared_ptr<Vector> vector, std::shared_ptr<Matrix> ans) {
  // Create vector of threads
  std::vector<std::thread> threads;

  int i = 0;
  do {
    // Calculate number of rows this thread is responsible for
    int rowsResponsible = i < SIZE % NUM_THREADS ? SIZE / NUM_THREADS + 1 : SIZE / NUM_THREADS;

    // Add thread to vector
    threads.emplace_back(matrixVectorMultiplication, matrix, vector, ans, i, rowsResponsible);

    i += rowsResponsible;
  } while (i < SIZE);

  // Await all threads to finish
  for (auto &thread : threads)
    thread.join();
}

int main() {
  // Initialize matrix and vector to be multiplied together
  auto matrix = std::make_shared<Matrix>(SIZE, true);
  auto vector = std::make_shared<Vector>(SIZE, false);

  // Initialize answer matrix to save answers
  auto ans = std::make_shared<Matrix>(SIZE, false);

  // Print starting matrix
  if(PRINT_MATRICES) printMatrix(matrix, "Beginning matrix");

  auto start = std::chrono::high_resolution_clock::now();

  // Perform calculations
  runCalculations(matrix, vector, ans);

  auto end = std::chrono::high_resolution_clock::now();

  // Print answer matrix
  if(PRINT_MATRICES) printMatrix(ans, "Answer");

  // Print time taken
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds";

  return 0;
}
