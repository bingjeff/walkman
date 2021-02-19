
// Timing - Static value used to match up with the music
// x and y are the values that gravity alternates between
// for each axis (between -1 and +1 multiplied by this value).
// They are dynamically updated.
var params = {
    timing: 675,
    x: 0,
    y: 0,
};

var Bodies = Matter.Bodies;
var Composites = Matter.Composites;
var Constraint = Matter.Constraint;
var Engine = Matter.Engine;
var World = Matter.World;

var engine = Engine.create({
    constraintIterations: 40, // default = 2
    positionIterations: 120, // default = 6
    velocityIterations: 80, // default = 4
    render: {
        element: document.getElementById("fiz-x"),
        options: {
            width: 800,
            height: 400,
            background: '#fafafa',
            wireframeBackground: '#222',
            hasBounds: false,
            enabled: true,
            wireframes: true,
            showSleeping: true,
            showDebug: false,
            showBroadphase: false,
            showBounds: false,
            showVelocity: false,
            showCollisions: true,
            showAxes: false,
            showPositions: false,
            showAngleIndicator: true,
            showIds: false,
            showShadows: false
        }
    }
});

var ellipse = function(r_wide, r_high, n_points) {
    var vertices = [];
    var angle_increment = 2.0 * Math.PI / n_points;
    for (var c = 0; c < n_points; c++) {
        var x = r_wide * Math.cos(c * angle_increment);
        var y = r_high * Math.sin(c * angle_increment);
        vertices.push(Matter.Vector.create(x, y));
    }
    return vertices;
}
var friction_static = 0.6;
var friction_dynamic = 0.5;
var num_ellipse_points = 200;
// create two boxes and a ground
var collisionGround = 0x0001,
    collisionLeftLeg = 0x0002,
    collisionRightLeg = 0x0004;
var man = Matter.Composite.create();
var rightShank = Bodies.rectangle(200, 100, 20, 200);
var rightFoot = Bodies.fromVertices(200, 200, ellipse(80, 40, num_ellipse_points));
var rightLeg = Matter.Body.create({
    parts: [rightShank, rightFoot],
    friction: friction_dynamic,
    frictionStatic: friction_static,
    collisionFilter: {
        category: collisionRightLeg,
        mask: collisionGround | collisionRightLeg
    }
});
Matter.Composite.add(man, rightLeg);
Matter.Composite.rotate(man, 0.05, {x:200, y:0});
var leftShank = Bodies.rectangle(200, 100, 20, 200);
var leftFoot = Bodies.fromVertices(200, 200, ellipse(80, 40, num_ellipse_points));
var leftLeg = Matter.Body.create({
    parts: [leftShank, leftFoot],
    friction: friction_dynamic,
    frictionStatic: friction_static,
    collisionFilter: {
        category: collisionLeftLeg,
        mask: collisionGround | collisionLeftLeg
    }
});
Matter.Composite.add(man, leftLeg);
Matter.Composite.add(man, Matter.Constraint.create({
    bodyA: rightLeg,
    bodyB: leftLeg,
    pointA: { x: 5, y: -140},
    pointB: { x: 0, y: -140},
    stiffness: 0.8,
}));
var ground = Bodies.rectangle(400, 350, 810, 60, {
    isStatic: true,
    friction: friction_dynamic,
    frictionStatic: friction_static,
    collisionFilter: {
        category: collisionGround,
        mask: collisionRightLeg | collisionLeftLeg
    }
});
Matter.Body.rotate(ground, 0.05);


// add all of the bodies to the world
World.add(engine.world, [man, ground]);

// run the engine
Engine.run(engine);
