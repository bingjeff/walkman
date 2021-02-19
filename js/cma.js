/* Covariance Matrix Adaptation (CMA) Optimization
 * adapted from: https://en.wikipedia.org/wiki/CMA-ES
 */
// TODO: Should convert to using MathJS instead of NumericJS
var frosenbrock = function(x) {
  var n = x.length;
  var xa = x.slice(0, n - 1);
  var xb = x.slice(1, n);
  return 100.0 *
    numeric.sum(numeric.pow(numeric.sub(numeric.pow(xa, 2), xb), 2)) +
    numeric.sum(numeric.pow(numeric.sub(xa, 1.0), 2));
}

// Adapted from: https://en.wikipedia.org/wiki/Box-Muller_transform
var randomNormal = function(mean, stddev) {
  var mu = mean,
      sigma = stddev,
      epsilon = Number.EPSILON,
      two_pi = 2.0 * Math.PI,
      z0 = 0.0,
      z1 = 0.0,
      generate = false;

  this.generate = function() {
    generate = !generate;
    if(!generate) {
      return z1 * sigma + mu;
    } else {
      var u1 = 0.0,
          u2 = 0.0;
      while(u1 <= epsilon) {
        u1 = Math.random();
        u2 = Math.random();
      }
      z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(two_pi * u2);
      z1 = Math.sqrt(-2.0 * Math.log(u1)) * Math.sin(two_pi * u2);
      return z0 * sigma + mu;
    }
  };
}

var randomNormalVector = function(numElements, mean, stddev) {
  var normalDist = new randomNormal(mean, stddev);
  var vec = [];
  for (var i = numElements - 1; i >= 0; i--) {
    vec.push(normalDist.generate());
  }
  return vec;
}

var cmaSolver = function(fitnessFunction, initialGuess, options) {
    options = options || {};
    var nm = numeric; // Shortener for numeric operators

    var evalFitness = fitnessFunction;
    var meanVars = initialGuess.slice(0);
    var bestVars = {
      fitness: Number.MAX_VALUE,
      bestGuess: initialGuess.slice(0)
    }
    var prevMeanVars = meanVars.slice(0);
    var numVars = meanVars.length;
    var settings = {
      sigma: 0.3, // Initial step-size (std. dev.)
      lambda: Math.floor(4 + 3 * Math.log(numVars)), // Population size
      muFraction: 0.5, // Fraction of population for recombination
      fitnessTolerance: 1.0e-6, // Stop after fitness is less than this
      maxNumEvaluations: 1e3 * numVars // Stop after this many function evals
    };

    for (var key in options) {
      settings[key] = options[key];
    }

    // Strategy parameter setting: Selection
    // step size, coordinate standard deviation
    var sigma = settings.sigma;
    // population size, offspring number
    var lambda = settings.lambda;
    // number of parents/points for recombination
    var mu = Math.floor(settings.muFraction * lambda);
    // [mu x 1] array for weighted recombination
    var weights = nm.add(
      Math.log(0.5 * lambda + 0.5),
      nm.mul(nm.log(nm.linspace(1, mu)), -1.0));
    weights = nm.div(weights, nm.sum(weights));
    // variance-effectiveness of sum w_i x_i
    var muW = Math.pow(nm.sum(weights), 2.0) / nm.sum(nm.pow(weights, 2.0));

    // Strategy parameter setting: Adaptation
    // time constant for cumulation for C
    var tauC = (4.0 + muW / numVars) / (numVars + 4.0 + 2.0 * muW / numVars);
    // time constant for cumulation of sigma control
    var tauSigma = (muW + 2.0) / (numVars + muW + 5.0);
    // time constant (learning rate) for rank-one update of C
    var tauOne = 2.0 / (Math.pow(numVars + 1.3, 2) + muW);
    // time constant (learning rate) for rank-mu update
    var tauMu = Math.min(
      1.0 - tauOne,
      2.0 * (muW - 2.0 + 1.0 / muW) / (Math.pow(numVars + 2, 2) + muW));
    // damping for sigma (usually close to 1)
    var damping = 1.0  + tauSigma + 2.0 * Math.max(0.0,
      Math.sqrt((muW - 1.0) / (numVars + 1.0)) - 1.0);

    // Initialize dynamic (internal) strategy parameters and constants
    // evolution paths for C
    var pathC = nm.rep([numVars], 0.0);
    // evolution paths for sigma
    var pathSigma = nm.rep([numVars], 0.0);
    // B defines the coordinate system
    var B = nm.identity(numVars);
    // diagonal D defines the scaling
    var D = nm.rep([numVars], 1.0);
    // covariance matrix C
    var C = nm.mul(B, nm.mul(nm.diag(nm.pow(D, 2)), nm.transpose(B)));
    // C^-1/2
    var invsqrtC = nm.mul(B, nm.mul(nm.diag(nm.div(1.0, D)), nm.transpose(B)));
    // track update of B and D
    var numEigenEvaluations = 0;
    // expectation of ||N(0,I)|| == norm(randn(N,1))
    var chiN = Math.sqrt(numVars) *
      (1.0 - 1 / (4.0 * numVars) + 1 / (21.0 * Math.pow(numVars,2)));

    // Generate and evaluate lambda offspring
    var populationVars = nm.rep([numVars, lambda], 0.0);
    var populationFitness = nm.rep([lambda, 2], 0.0);
    var numEvaluations = 0;

   this.generatePopulation = function() {
      for (var k = lambda - 1; k >= 0; k--) {
        // meanVars + sigma * Normal(0,C)
        var individual = nm.add(meanVars,
          nm.mul(sigma,
            nm.dot(B, nm.mul(D, randomNormalVector(numVars, 0.0, 1.0)))));
        for (var j = numVars - 1; j >= 0; j--) {
          populationVars[j][k] = individual[j];
        }
      }
    };
    this.getPopulation = function() {
      return populationVars;
    }
    this.evaluateFitness = function() {
      for (var k = lambda - 1; k >= 0; k--) {
        populationFitness[k][0] = evalFitness(individual);
        populationFitness[k][1] = k;
        ++numEvaluations;
      }
    };
    this.getFitness = function() {
      return populationFitness;
    }
    this.updateEvolution = function() {
      // Sort by fitness and compute weighted mean storing in meanVars
      populationFitness.sort(function(a, b){return a[0] - b[0];});
      prevMeanVars = meanVars.slice(0);
      for (var j = numVars - 1; j >= 0; j--) {
        meanVars[j] = 0;
        for (var k = 0; k < mu; k++) {
          sortedK = populationFitness[k][1];
          meanVars[j] += populationVars[j][sortedK] * weights[k];
        }
        // Bias with the best guess so far
        this.getBestGuess();
        biasBestGuess = 0.0;
        meanVars[j] = biasBestGuess * bestVars.bestGuess[j] +
                      (1 - biasBestGuess) * meanVars[j];
      }

      // Cumulation: Update evolution paths
      // ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma
      pathSigma = nm.add(nm.mul(1 - tauSigma, pathSigma),
            nm.dot(
              nm.mul(Math.sqrt(tauSigma * (2.0 - tauSigma) * muW) / sigma, invsqrtC),
              nm.sub(meanVars, prevMeanVars)));
      // hsig = norm(ps)/sqrt(1-(1-cs)^(2*counteval/lambda))/chiN < 1.4 + 2/(N+1)
      var hSigmaLHS = (nm.norm2(pathSigma) /
        Math.sqrt(1.0 - Math.pow(1.0 - tauSigma, 2.0 * numEvaluations / lambda)) /
        chiN);
      var hSigmaRHS = 1.4 + 2.0 / (numVars + 1.0);
      var hSigma = (hSigmaLHS < hSigmaRHS) ? 1.0 : 0.0;
      // pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma
      pathC = nm.add(nm.mul(1 - tauC, pathC),
            nm.mul(
              Math.sqrt(tauC * (2.0 - tauC) * muW) * hSigma / sigma,
              nm.sub(meanVars, prevMeanVars)));

      // Adapt covariance matrix C
      var discountedC = nm.mul(1-tauOne-tauMu, C);
      var rankOneUpdateCorrection = nm.mul((1.0 - hSigma) * tauC * (2.0 - tauC), C);
      var rankOneUpdate = nm.mul(tauOne, nm.add(
        nm.tensor(pathC, pathC),
        rankOneUpdateCorrection));
      var rankMuVars = nm.rep([numVars, mu], 0);
      for (var j = numVars - 1; j >= 0; j--) {
        for (var k = 0; k < mu; k++) {
          sortedK = populationFitness[k][1];
          rankMuVars[j][k] = (populationVars[j][sortedK] - prevMeanVars[j]) / sigma;
        }
      }
      var rankMuUpdate = nm.mul(tauMu,
        nm.dot(rankMuVars,
          nm.dot(nm.diag(weights), nm.transpose(rankMuVars))));
      C =  nm.add(discountedC, rankOneUpdate, rankMuUpdate);

      // Adapt step size sigma
      sigma = sigma * Math.exp((tauSigma / damping) *
        (nm.norm2(pathSigma) / chiN - 1.0));

      // Decomposition of C into B*diag(D.^2)*B' (diagonalization)
      if ((numEvaluations - numEigenEvaluations) >
        (0.1 * lambda / (tauOne + tauMu) / numVars )) {
          numEigenEvaluations = numEvaluations;
          // enforce symmetry
          for (var j = 1; j < numVars; j++) {
            for (var k = 0; k < j; k++) {
              C[j][k] = C[k][j];
            }
          }
          var svdC = nm.svd(C); // eigen decomposition (use SVD as a stand-in)
          B = svdC.U; // normalized eigenvectors
          D = nm.sqrt(svdC.S); // vector of standard deviations
          invsqrtC = nm.dot(B, nm.dot(nm.diag(nm.div(1.0, D)), nm.transpose(B)));
      }
    };
    this.shouldTerminate = function() {
      return (populationFitness[0][0] <= settings.fitnessTolerance) ||
        (numEvaluations > settings.maxNumEvaluations);
    };
    this.getBestGuess = function() {
      var sortedK = populationFitness[0][1];
      var guesses = nm.transpose(populationVars);
      if (bestVars.fitness > populationFitness[0][0]) {
        bestVars.fitness = populationFitness[0][0];
        bestVars.bestGuess = guesses[sortedK].slice(0);
      }
      return bestVars;
    };
    this.solve = function() {
      while(!this.shouldTerminate()) {
        this.generatePopulation();
        this.evaluateFitness();
        this.updateEvolution();
      }
    }

}
