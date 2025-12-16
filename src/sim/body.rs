use glam::Vec2;
use particular::prelude::{Particle, VectorDescriptor};
use rapier2d::prelude::*;


use crate::sim::{GRAVITY_AMPLIFIER, UNIVERSAL_GRAVITY};

#[derive(Default)]
pub struct Body {
    position: Vec2,
    rotation: f32,
    radius: f32,
    mass: f32,
    pub rigidbody_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
}

impl Particle for Body {
    type Vector = VectorDescriptor<2, [f32; 2]>;

    fn position(&self) -> [f32; 2] {
        [self.position.x, self.position.y]
    }

    fn mu(&self) -> f32 {
        self.mass * GRAVITY_AMPLIFIER * UNIVERSAL_GRAVITY
    }
}

impl Body {
    pub fn new(
        rigidbody_handle: RigidBodyHandle,
        collider_handle: ColliderHandle,
    ) -> Self {
        Self {
            rigidbody_handle,
            collider_handle,
            ..Default::default()
        }
    }
}

impl Body {
    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn rotation(&self) -> f32 {
        self.rotation
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn sync_to_rigidbody(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        let rb = bodies.get(self.rigidbody_handle).unwrap();
        let coll = colliders.get(self.collider_handle).unwrap();

        let translation = rb.translation();
        self.position = Vec2::new(translation.x, translation.y);
        self.rotation = rb.rotation().angle();
        self.radius = coll.shape().as_ball().unwrap().radius;
        self.mass = rb.mass();
    }

    pub fn apply_acceleration_to_rigidbody(
        &self,
        bodies: &mut RigidBodySet,
        acceleration: [f32; 2],
    ) {
        let rb = bodies.get_mut(self.rigidbody_handle).unwrap();
        let force = Vec2::new(acceleration[0], acceleration[1]) * self.mass();

        rb.reset_forces(true);
        rb.add_force(nalgebra::Vector2::new(force.x, force.y), true);
    }
}
