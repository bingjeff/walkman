var ellipse = function(r_wide, r_high, n_points) {
  var vertices = [];
  var angle_increment = 2.0 * Math.PI / n_points;
  for (var c = 0; c < n_points; c++) {
    var x = r_wide * Math.cos(c * angle_increment);
    var y = r_high * Math.sin(c * angle_increment);
    vertices.push([x, y]);
  }
  return vertices;
}

var passiveWalker = function(options) {
    options = options || {};

    var settings = {
      legLength: 0.2,
      footWidth: 0.05,
      footHeight: 0.03,
      footRotation: 0.0,
      footResolution: 40,
      bellySize: 0.04,
      bellyOffsetY: 0.04,
      bellyOffsetX: 0.004,
      initHeight: 0.15,
      bodyDensity: 100.0,
      constraintGroupLeg: Math.pow(2, 1),
      constraintGroupGround: Math.pow(2, 2),
      materialFoot: new p2.Material(),
      swingAngle: 0.1
    };

    for (var key in options) {
      settings[key] = options[key];
    }

    function makeLeg() {

      var shank = new p2.Box({
        width: 0.1 * settings.legLength,
        height: settings.legLength,
        collisionGroup: settings.constraintGroupLeg,
        collisionMask: settings.constraintGroupGround
      });
      var belly = new p2.Circle({
        radius: 0.5 * settings.bellySize,
        collisionGroup: Math.pow(2, 3),
        collisionMask: Math.pow(2, 4)
      });
      var foot = new p2.Convex({
        vertices: ellipse(settings.footWidth, settings.footHeight, settings.footResolution),
        collisionGroup: settings.constraintGroupLeg,
        collisionMask: settings.constraintGroupGround,
        material: settings.materialFoot
      });
      var leg = new p2.Body({
        position: [0, settings.initHeight],
        mass: 1
      });
      leg.addShape(shank, [0, 0.5 * settings.legLength], 0);
      leg.addShape(belly, [settings.bellyOffsetX, settings.bellyOffsetY], 0);
      leg.addShape(foot, [0.0, 0.0], settings.footRotation);
      leg.setDensity(settings.bodyDensity);
      leg.adjustCenterOfMass();

      return leg;
    }

    this.rightLeg = makeLeg();
    this.leftLeg = makeLeg();
    this.hipJoint = new p2.RevoluteConstraint(this.rightLeg, this.leftLeg, {
      worldPivot: [0.0, settings.initHeight + settings.legLength]
    });

    var swingLeg = this.rightLeg;
    this.getSwingLeg = function() {
      if (this.hipJoint.angle > settings.swingAngle) {
        swingLeg = this.leftLeg;
      }
      if (this.hipJoint.angle < -settings.swingAngle) {
        swingLeg = this.rightLeg;
      }
      return swingLeg;
    }
    this.getStanceLeg = function() {
      if (swingLeg === this.rightLeg) {
        return this.leftLeg;
      } else {
        return this.rightLeg;
      }
    }
    this.hasFallen = function() {
      var stanceLeg = this.getStanceLeg();
      var footPosition = [0, 0];
      var hipPosition = [0, 0];
      var footPositionLocal = stanceLeg.shapes[2].position;
      var hipPositionLocal = [footPositionLocal[0],
                              footPositionLocal[1] + settings.legLength];
      stanceLeg.toWorldFrame(footPosition, footPositionLocal);
      stanceLeg.toWorldFrame(hipPosition, hipPositionLocal);
      var hipHeight = hipPosition[1] - footPosition[1];
      if (hipHeight < 0.5 * settings.legLength) {
        return true;
      } else {
        return false;
      }
    }
    this.getDistanceWalked = function() {
      var hipPosition = [0, 0];
      var hipPositionLocal = [swingLeg.shapes[2].position[0],
                              swingLeg.shapes[2].position[1] + settings.legLength];
      swingLeg.toWorldFrame(hipPosition, hipPositionLocal);
      return hipPosition[0];
    }
  }
  passiveWalker.prototype.addToWorld = function(world) {
    world.addBody(this.rightLeg);
    world.addBody(this.leftLeg);
    world.addConstraint(this.hipJoint);
  }
  passiveWalker.prototype.removeFromWorld = function(world) {
    world.removeConstraint(this.hipJoint);
    world.removeBody(this.rightLeg);
    world.removeBody(this.leftLeg);
  }

  // Create demo application
var app = new p2.WebGLRenderer(function() {

  // Create the physics world
  var world = new p2.World({
    gravity: [0, -10]
  });

  // Register the world in the demo app
  this.setWorld(world);

  // Set stiffness of contact & constraints
  world.setGlobalStiffness(1e4);

  world.solver.iterations = 100;
  world.solver.tolerance = 0.001;
  world.islandSplit = true;

  // Enable dynamic friction. A bit more expensive than without, but gives more
  // accurate friction
  world.solver.frictionIterations = 20;

  // Create constraint groups
  var constraintGroupLeg = Math.pow(2, 1),
    constraintGroupGround = Math.pow(2, 2);
  // Create material groups
  var materialRubber = new p2.Material();

  // Create ground
  var planeShape = new p2.Plane();
  planeShape.collisionGroup = constraintGroupGround;
  planeShape.collisionMask = constraintGroupLeg;
  planeShape.material = materialRubber;
  var plane = new p2.Body({
    mass: 0, // static
    position: [0.0, 0.0],
    angle: 0.05
  });
  plane.addShape(planeShape);
  world.addBody(plane);

  // Add a contact material
  var contactMaterialRubber = new p2.ContactMaterial(materialRubber, materialRubber, {
    friction: 0.8
  });
  world.addContactMaterial(contactMaterialRubber);

  // Create the CMA solver object
  var cma = new cmaSolver(
    function() {return 0;},
    [
      0.03, // footWidth
      0.04, // bellySize
      0.04, // bellyOffsetY
      0.00, // bellyOffsetX
      0.10  // swingAngle
    ],
    {
      sigma: 0.005,
      lambda: 32, // Nominally is 8
      muFraction: 0.5 // Nominally is 0.5
    });

  // Create storage for walkers
  var walkers = [];
  // Create a time for the walker
  var timeDuration = 5.0;
  var timeNext = world.time + timeDuration;

  var updatePopulation = function () {
    walkers.length = 0;
    timeNext = world.time + timeDuration;
    cma.generatePopulation();
    var population = cma.getPopulation();
    for (var c = population[0].length - 1; c >= 0; c--) {
      walkers.push(new passiveWalker({
        footWidth: Math.abs(population[0][c]),
        bellySize: Math.abs(population[1][c]),
        bellyOffsetY: Math.abs(population[2][c]),
        bellyOffsetX: population[3][c],
        constraintGroupLeg: constraintGroupLeg,
        constraintGroupGround: constraintGroupGround,
        materialFoot: materialRubber,
        swingAngle: Math.abs(population[4][c])
      }));
    }
    for (var c = walkers.length - 1; c >= 0; c--) {
      walkers[c].addToWorld(world);
    }
  };

  var updateFitness = function() {
    var fitness = cma.getFitness();
    for (var c = walkers.length - 1; c >= 0; c--) {
      fitness[c][1] = c;
      fitness[c][0] = 0.1*(timeNext - world.time);
      if (!walkers[c].hasFallen()) {
        fitness[c][0] += walkers[c].getDistanceWalked();
      } else {
        fitness[c][0] += 0.1;
      }
    }
  }

  // Deal with contact of bodies
  world.on("postBroadphase", function(evt) {
    var numPairs = evt.pairs.length / 2;
    for (var c = 0; c < numPairs; c++) {
      bodyA = evt.pairs.shift();
      bodyB = evt.pairs.shift();
      var isContactSwingleg = false;
      for (var w = walkers.length - 1; w >= 0; w--) {
        isContactSwingleg |= (bodyA === walkers[w].getSwingLeg() ||
        bodyB === walkers[w].getSwingLeg());
      }
      if (!isContactSwingleg) {
        evt.pairs.push(bodyA);
        evt.pairs.push(bodyB);
      }
    }
  });

  // Deal with walker falling over
  world.on("postStep", function(evt) {
    updateFitness();
    allHaveFallen = true;
    for (var c = walkers.length - 1; c >= 0; c--) {
      allHaveFallen &= walkers[c].hasFallen();
    }
    if (world.time > timeNext || allHaveFallen) {
      cma.updateEvolution();
      for (var c = walkers.length - 1; c >= 0; c--) {
        walkers[c].removeFromWorld(world);
      }
      var result = cma.getBestGuess();
      console.log("Fitness: " + result.fitness);
      updatePopulation();
    }
  });

  updatePopulation();

  // Setup render settings and time-stepping
  this.settings.fps = 500;
  this.settings.maxSubSteps = 10;
  // Set camera position and zoom
  this.frame(0, 0, 2, 1);

}, {
  lineWidth: 0.001
});
