#pragma once

namespace funkit::math {

/**
 * Calculator
 *
 * A templated class that allows for calculations
 * @tparam I - the inputs of the calculator
 * @tparam O - the outputs of the calculator
 * @tparam C - any necessary constants
 */
template <typename I, typename O, typename C> class Calculator {
public:
  void setConstants(C constants) { constants_ = constants; }

  /**
   * calculate()
   *
   * Calculates values
   * @param input - calculator inputs
   */
  virtual O calculate(I input) = 0;

protected:
  C constants_;
};

/**
 * IterativeCalculator
 *
 * A templated class that allows for iterative calculations, mimicking recursion
 * @tparam I - the inputs of the calculator
 * @tparam O - the outputs of the calculator
 * @tparam C - any necessary constants
 */
template <typename I, typename O, typename C>
class IterativeCalculator : public Calculator<I, O, C> {
protected:
  /**
   * calculateIteration()
   *
   * Calculates an output based on current inputs and previous outputs
   */
  virtual O calculateIteration(I input, O prev_output) = 0;

public:
  /**
   * calculate()
   *
   * Iteratively calculates in a recursive-like manner.
   * @param input - the calculator input
   */
  O calculate(I input) override final {
    O output = O{};
    for (int i = 0; i < max_iterations; i++) {
      output = calculateIteration(input, output);
    }
    return output;
  }

  /**
   * setMaxIterations()
   *
   * @param the max possible iterations of the calculator
   */
  void setMaxIterations(int max_iterations) {
    this->max_iterations = max_iterations;
  }

protected:
  int max_iterations = 7;
};

}  // namespace funkit::math