#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ExPhysicsEngine_hpp
#define airsim_core_ExPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>

/* добавляем логирование*/
#include "common/LogFileWriter.hpp"
#include "vehicles/plane/Plane.hpp"

namespace msr {
	namespace airlib {

		class ExPhysicsEngine : public PhysicsEngineBase {
		public:
			ExPhysicsEngine(bool enable_ground_lock = true)
				: enable_ground_lock_(enable_ground_lock)
			{
				std::string filename = std::string("J:/Unreal/LogAirSim/") + plane_logger_name_ + std::string("_PhysicsEngine.txt");
				this->plane_logger_.open(filename, true);
				//enable_ground_lock_ = false;
			}

			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				for (PhysicsBody* body_ptr : *this) {
					initPhysicsBody(body_ptr);
				}
			}

			virtual void insert(PhysicsBody* body_ptr) override
			{
				PhysicsEngineBase::insert(body_ptr);

				initPhysicsBody(body_ptr);
			}

			virtual void update() override
			{
				PhysicsEngineBase::update();

				for (PhysicsBody* body_ptr : *this) {
					updatePhysics(*body_ptr);
				}
			}
			virtual void reportState(StateReporter& reporter) override
			{
				for (PhysicsBody* body_ptr : *this) {
					reporter.writeValue("Phys", debug_string_.str());
					reporter.writeValue("Is Gounded", body_ptr->isGrounded());
					reporter.writeValue("Force (world)", body_ptr->getWrench().force);
					reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
				}
				//call base
				UpdatableObject::reportState(reporter);
			}
			//*** End: UpdatableState implementation ***//

		private:
			void initPhysicsBody(PhysicsBody* body_ptr)
			{
				body_ptr->last_kinematics_time = clock()->nowNanos();
			}

			void updatePhysics(PhysicsBody& body)
			{
				TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);

				//get current kinematics state of the body - this state existed since last dt seconds
				const Kinematics::State& current = body.getKinematics();
				Kinematics::State next;
				Wrench next_wrench;
				plane_logger_.write("ORIENTATION \t >>>> [");
				plane_logger_.write(current.pose.orientation);
				plane_logger_.write("]\n");
				plane_logger_.write("VELOCITY \t >>>> [");
				plane_logger_.write(current.twist.linear);
				plane_logger_.write("]\n");
				plane_logger_.write("POSITION \t >>>> [");
				plane_logger_.write(current.pose.position);
				plane_logger_.write("]\n");
				plane_logger_.write("ACC  LIN \t >>>> [");
				plane_logger_.write(current.accelerations.linear);
				plane_logger_.write("]\n");
				plane_logger_.write("ACC  ANG \t >>>> [");
				plane_logger_.write(current.accelerations.angular);
				plane_logger_.write("]\n");

				//first compute the response as if there was no collision
				//this is necessary to take in to account forces and torques generated by body
				getNextKinematicsNoCollision(dt, body, current, next, next_wrench, plane_logger_);

				//if there is collision, see if we need collision response
				const CollisionInfo collision_info = body.getCollisionInfo();
				CollisionResponse& collision_response = body.getCollisionResponseInfo();
				//if collision was already responded then do not respond to it until we get updated information
				if (body.isGrounded() || (collision_info.has_collided && collision_response.collision_time_stamp != collision_info.time_stamp)) {
					plane_logger_.write("Get ON Collision\n");
					bool is_collision_response = getNextKinematicsOnCollision(dt, collision_info, body,
						current, next, next_wrench, enable_ground_lock_, plane_logger_);
					updateCollisionResponseInfo(collision_info, next, is_collision_response, collision_response);
					//throttledLogOutput("*** has collision", 0.1);
				}
				//else throttledLogOutput("*** no collision", 0.1);

				//Utils::log(Utils::stringf("T-VEL %s %" PRIu64 ": ", 
				//    VectorMath::toString(next.twist.linear).c_str(), clock()->getStepCount()));

				body.setWrench(next_wrench);
				plane_logger_.write("KINEMATICS\n_____________________\n");
				plane_logger_.write("next twist linear (");
				plane_logger_.write(next.twist.linear);
				plane_logger_.write(")\n");
				plane_logger_.write("next twist angular (");
				plane_logger_.write(next.twist.angular);
				plane_logger_.write(")\n______________________\n");
				body.updateKinematics(next);


				//TODO: this is now being done in PawnSimApi::update. We need to re-think this sequence
				//with below commented out - Arducopter GPS may not work.
				//body.getEnvironment().setPosition(next.pose.position);
				//body.getEnvironment().update();

			}

			static void updateCollisionResponseInfo(const CollisionInfo& collision_info, const Kinematics::State& next,
				bool is_collision_response, CollisionResponse& collision_response)
			{
				collision_response.collision_time_stamp = collision_info.time_stamp;
				++collision_response.collision_count_raw;

				//increment counter if we didn't collided with high velocity (like resting on ground)
				if (is_collision_response && next.twist.linear.squaredNorm() > kRestingVelocityMax * kRestingVelocityMax)
					++collision_response.collision_count_non_resting;

			}

			//return value indicates if collision response was generated
			static bool getNextKinematicsOnCollision(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
				const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock, LogFileWriter & logger)
			{
				/************************* Collision response ************************/
				const real_T dt_real = static_cast<real_T>(dt);
				logger.write("on collision normal ");
				logger.write(collision_info.normal);
				logger.write("\nwrench force");
				logger.write(next_wrench.force);
				logger.write("\ncurrent.accelerations.linear");
				logger.write(next.accelerations.linear);
				logger.write("\ncurrent.accelerations.angular");
				logger.write(next.accelerations.angular);
				logger.endl();
				
				//are we going away from collision? if so then keep using computed next state
				if ((collision_info.normal.dot(next.twist.linear) > 0.0f))
					return false;

				/********** Core collision response ***********/
				//get avg current velocity
				const Vector3r vcur_avg = current.twist.linear + current.accelerations.linear * dt_real;

				//get average angular velocity
				const Vector3r angular_avg = current.twist.angular + current.accelerations.angular * dt_real;

				//contact point vector
				Vector3r r = collision_info.impact_point - collision_info.position;

				//see if impact is straight at body's surface (assuming its box)
				const Vector3r normal_body = VectorMath::transformToBodyFrame(collision_info.normal, current.pose.orientation);
				const bool is_ground_normal = Utils::isApproximatelyEqual(std::abs(normal_body.z()), 1.0f, kAxisTolerance);
				
				bool ground_collision = false;
				const float z_vel = vcur_avg.z();
				const bool is_landing = z_vel > std::abs(vcur_avg.x()) && z_vel > std::abs(vcur_avg.y());

				real_T restitution = body.getRestitution();
				real_T friction = body.getFriction();

				if (is_ground_normal && is_landing
					// So normal_body is the collision normal translated into body coords, why does an x==1 or y==1
					// mean we are coliding with the ground???
					// || Utils::isApproximatelyEqual(std::abs(normal_body.x()), 1.0f, kAxisTolerance) 
					// || Utils::isApproximatelyEqual(std::abs(normal_body.y()), 1.0f, kAxisTolerance) 
					) {
					logger.write("is ground normal and is landing\n");
					// looks like we are coliding with the ground.  We don't want the ground to be so bouncy
					// so we reduce the coefficient of restitution.  0 means no bounce.
					// TODO: it would be better if we did this based on the material we are landing on.
					// e.g. grass should be inelastic, but a hard surface like the road should be more bouncy.
					restitution = 0;
					// crank up friction with the ground so it doesn't try and slide across the ground
					// again, this should depend on the type of surface we are landing on.
					friction = 1;

					//we have collided with ground straight on, we will fix orientation later
					ground_collision = is_ground_normal;
				}

				//velocity at contact point
				const Vector3r vcur_avg_body = VectorMath::transformToBodyFrame(vcur_avg, current.pose.orientation);
				const Vector3r contact_vel_body = vcur_avg_body + angular_avg.cross(r);

				/*
					GafferOnGames - Collision response with columb friction
					http://gafferongames.com/virtual-go/collision-response-and-coulomb-friction/
					Assuming collision is with static fixed body,
					impulse magnitude = j = -(1 + R)V.N / (1/m + (I'(r X N) X r).N)
					Physics Part 3, Collision Response, Chris Hecker, eq 4(a)
					http://chrishecker.com/images/e/e7/Gdmphys3.pdf
					V(t+1) = V(t) + j*N / m
				*/
				const real_T impulse_mag_denom = 1.0f / body.getMass() +
					(body.getInertiaInv() * r.cross(normal_body))
					.cross(r)
					.dot(normal_body);
				const real_T impulse_mag = -contact_vel_body.dot(normal_body) * (1 + restitution) / impulse_mag_denom;

				next.twist.linear = vcur_avg + collision_info.normal * (impulse_mag / body.getMass());
				next.twist.angular = angular_avg + r.cross(normal_body) * impulse_mag;

				//above would modify component in direction of normal
				//we will use friction to modify component in direction of tangent
				const Vector3r contact_tang_body = contact_vel_body - normal_body * normal_body.dot(contact_vel_body);
				const Vector3r contact_tang_unit_body = contact_tang_body.normalized();
				const real_T friction_mag_denom = 1.0f / body.getMass() +
					(body.getInertiaInv() * r.cross(contact_tang_unit_body))
					.cross(r)
					.dot(contact_tang_unit_body);
				const real_T friction_mag = -contact_tang_body.norm() * friction / friction_mag_denom;

				const Vector3r contact_tang_unit = VectorMath::transformToWorldFrame(contact_tang_unit_body, current.pose.orientation);
				next.twist.linear += contact_tang_unit * friction_mag;
				next.twist.angular += r.cross(contact_tang_unit_body) * (friction_mag / body.getMass());

				//TODO: implement better rolling friction
				next.twist.angular *= 0.9f;

				// there is no acceleration during collision response, this is a hack, but without it the acceleration cancels
				// the computed impulse response too much and stops the vehicle from bouncing off the collided object.
				logger.write("before ZERO\nnext acc linear ");
				logger.write(next.accelerations.linear);
				logger.write("\nnext acc angular ");
				logger.write(next.accelerations.angular);
				logger.endl();
				next.accelerations.linear = Vector3r::Zero();
				next.accelerations.angular = Vector3r::Zero();

				next.pose = current.pose;
				if (enable_ground_lock && ground_collision) {
					logger.write("enabled ground lock AND ground collision\n");
					float pitch, roll, yaw;
					VectorMath::toEulerianAngle(next.pose.orientation, pitch, roll, yaw);
					pitch = roll = 0;
					next.pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);

					//there is a lot of random angular velocity when vehicle is on the ground
					next.twist.angular = Vector3r::Zero();

					// also eliminate any linear velocity due to twist - since we are sitting on the ground there shouldn't be any.
					next.twist.linear = Vector3r::Zero();
					next.pose.position = collision_info.position;
					body.setGrounded(true);

					// but we do want to "feel" the ground when we hit it (we should see a small z-acc bump)
					// equal and opposite our downward velocity.
					next.accelerations.linear = -0.5f * body.getMass() * vcur_avg;

					//throttledLogOutput("*** Triggering ground lock", 0.1);
				}
				else
				{
					//else keep the orientation
					logger.write("keep orientation\n");
					next.pose.position = collision_info.position + (collision_info.normal * collision_info.penetration_depth) + next.twist.linear * (dt_real * kCollisionResponseCycles);
				}
				next_wrench = Wrench::zero();

				//Utils::log(Utils::stringf("*** C-VEL %s: ", VectorMath::toString(next.twist.linear).c_str()));

				return true;
			}

			void throttledLogOutput(const std::string& msg, double seconds)
			{
				TTimeDelta dt = clock()->elapsedSince(last_message_time);
				const real_T dt_real = static_cast<real_T>(dt);
				if (dt_real > seconds)
				{
					Utils::log(msg);
					last_message_time = clock()->nowNanos();
				}
			}

			static Wrench getDragWrench(const PhysicsBody& body, const Quaternionr& orientation,
				const Vector3r& linear_vel, const Vector3r& angular_vel_body)
			{
				//add linear drag due to velocity we had since last dt seconds
				//drag vector magnitude is proportional to v^2, direction opposite of velocity
				//total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
				//To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
				//http://physics.stackexchange.com/questions/304742/angular-drag-on-body
				//similarly calculate angular drag
				//note that angular velocity, acceleration, torque are already in body frame

				Wrench wrench = Wrench::zero();
				const real_T air_density = body.getEnvironment().getState().air_density;

				for (uint vi = 0; vi < body.dragVertexCount(); ++vi) {
					const auto& vertex = body.getDragVertex(vi);
					const Vector3r vel_vertex = VectorMath::transformToBodyFrame(linear_vel, orientation) + angular_vel_body.cross(vertex.getPosition());
					const real_T vel_comp = vertex.getNormal().dot(vel_vertex);
					//if vel_comp is -ve then we cull the face. If velocity too low then drag is not generated
					if (vel_comp > kDragMinVelocity) {
						const Vector3r drag_force = vertex.getNormal() * (-vertex.getDragFactor() * air_density * vel_comp * vel_comp);
						const Vector3r drag_torque = vertex.getPosition().cross(drag_force);

						wrench.force += drag_force;
						wrench.torque += drag_torque;
					}
				}

				//convert force to world frame, leave torque to local frame
				wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

				return wrench;
			}

			static Wrench getBodyWrench(const PhysicsBody& body, const Quaternionr& orientation, LogFileWriter & logger)
			{
				//set wrench sum to zero
				Wrench wrench = Wrench::zero();

				/*if (isFullLogging_)
				{

				}*/
				if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
					logger.write("wrench");

				//calculate total force on rigid body's center of gravity
				for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
					//aggregate total
					const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
					const auto& vertex_wrench = vertex.getWrench();
					wrench += vertex_wrench;

					//add additional torque due to force applies farther than COG
					// tau = r X F
					wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
					if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
					{
						logger.write(wrench.force);
						logger.write(wrench.torque);
					}
				}
				if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
					logger.endl();
				//convert force to world frame, leave torque to local frame
				wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);
				if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
				{
					logger.write("[PLANE WRENCH]\nforce");
					logger.write(wrench.force);
					logger.write("torque");
					logger.write(wrench.torque);
					logger.endl();
				}
				return wrench;
			}
			static float updateFriction(float force, float friction)
			{
				if (force < 0) {
					return ((force + friction) < 0) ? (force + friction) : 0.0f;
				}
				else {
					return ((force - friction) > 0) ? (force - friction) : 0.0f;
				}
			}
			static bool getNextKinematicsOnCollision2(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
				const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock, LogFileWriter & logger)
			{
				const float vertical_velocity_collision = collision_info.normal.dot(next.twist.linear);
				const bool is_horizontal_ground = Utils::isApproximatelyEqual(std::abs(collision_info.normal.z()), 1.0f, kAxisTolerance);
				if (vertical_velocity_collision > 0.0f)
					return false;
				const real_T dt_real = static_cast<real_T>(dt);
				const Wrench body_wrench = next_wrench;
				//рычаг коллизии к центру масс
				Vector3r r = collision_info.impact_point - collision_info.position;
				//пересчитываем все силы и моменты: приложенные силы к объекту, реакции опоры, трение
				const Vector3r norm_world = Vector3r(0, 0, 1);
				const Eigen::Quaternionf q2collision = VectorMath::toQuaternion(norm_world, collision_info.normal);
				//const Eigen::Quaternionf q2world = VectorMath::toQuaternion(collision_info.normal, norm_world);
				Vector3r sum_force_world = body_wrench.force + body_wrench.torque.cross(r) 
												+ body.getEnvironment().getState().gravity*body.getMass();
				Vector3r sum_f_c = VectorMath::transformToBodyFrame(sum_force_world,q2collision);
				float reaction = (sum_f_c.z() > 0)? 0:(-sum_f_c.z());
				float friction = 0.5*reaction;
				float x_f_c = updateFriction(sum_f_c.x(), friction);
				float y_f_c = updateFriction(sum_f_c.y(), friction);
				Vector3r res_sum = Vector3r(x_f_c, y_f_c, 0.0f);
				Vector3r acc_sum = VectorMath::transformToWorldFrame(res_sum, q2collision) / body.getMass();

				if (body.isGrounded()) //<- взлет или посадка
				{

				}
					const Vector3r vcur_avg = current.twist.linear + current.accelerations.linear * dt_real;
					const Vector3r angular_avg = current.twist.angular + current.accelerations.angular * dt_real;
					

					//body frame
					const Vector3r normal_body = VectorMath::transformToBodyFrame(collision_info.normal, current.pose.orientation);
					const Vector3r vcur_avg_body = VectorMath::transformToBodyFrame(vcur_avg, current.pose.orientation);
					const Vector3r gravity_body = VectorMath::transformToBodyFrame(body.getEnvironment().getState().gravity, current.pose.orientation);

					bool ground_collision = false;
					const float z_vel = vcur_avg.z();
					real_T restitution = body.getRestitution();
					real_T friction = body.getFriction();
					const bool is_ground_normal = Utils::isApproximatelyEqual(std::abs(normal_body.z()), 1.0f, kAxisTolerance);
				//	const bool is_landing = z_vel > std::abs(vcur_avg.x()) && z_vel > std::abs(vcur_avg.y()); // <- бессмысленно для самолета
				//if (is_ground_normal && is_landing) {
				//	restitution = 0;
				//	friction = 1;
				//	ground_collision = is_ground_normal;
				//}
					
					const Vector3r contact_vel_body = vcur_avg_body + angular_avg.cross(r);
					const real_T impulse_mag_denom = 1.0f / body.getMass() +
						(body.getInertiaInv() * r.cross(normal_body))
						.cross(r)
						.dot(normal_body);
					const real_T impulse_mag = -contact_vel_body.dot(normal_body) * (1 + restitution) / impulse_mag_denom;
					next.twist.linear = vcur_avg + collision_info.normal * (impulse_mag / body.getMass());
					next.twist.angular = angular_avg + r.cross(normal_body) * impulse_mag;
					const Vector3r contact_tang_body = contact_vel_body - normal_body * normal_body.dot(contact_vel_body);
					const Vector3r contact_tang_unit_body = contact_tang_body.normalized();
					const real_T friction_mag_denom = 1.0f / body.getMass() +
						(body.getInertiaInv() * r.cross(contact_tang_unit_body))
						.cross(r)
						.dot(contact_tang_unit_body);
					const real_T friction_mag = -contact_tang_body.norm() * friction / friction_mag_denom;
					const Vector3r contact_tang_unit = VectorMath::transformToWorldFrame(contact_tang_unit_body, current.pose.orientation);
					next.twist.linear += contact_tang_unit * friction_mag;
					next.twist.angular += r.cross(contact_tang_unit_body) * (friction_mag / body.getMass());
					//TODO: implement better rolling friction
					next.twist.angular *= 0.9f;
					next.accelerations.linear = Vector3r::Zero();
					next.accelerations.angular = Vector3r::Zero();
					next.pose = current.pose;
					if (enable_ground_lock && ground_collision) {
						float pitch, roll, yaw;
						VectorMath::toEulerianAngle(next.pose.orientation, pitch, roll, yaw);
						pitch = roll = 0;
						next.pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);
						next.twist.angular = Vector3r::Zero();
						next.twist.linear = Vector3r::Zero();
						next.pose.position = collision_info.position;
						body.setGrounded(true);
						next.accelerations.linear = -0.5f * body.getMass() * vcur_avg;
					}
					else
					{
						next.pose.position = collision_info.position + (collision_info.normal * collision_info.penetration_depth) + next.twist.linear * (dt_real * kCollisionResponseCycles);
					}
					next_wrench = Wrench::zero();
					return true;
			}
			static void getNextKinematicsNoCollision2(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
				Kinematics::State& next, Wrench& next_wrench, LogFileWriter & logger)
			{
				const real_T dt_real = static_cast<real_T>(dt);
				const Wrench body_wrench = next_wrench;
				float normalizedForce = body_wrench.force.squaredNorm();
				float normalizedGravity = body.getEnvironment().getState().gravity.squaredNorm();
				Vector3r avg_linear = Vector3r::Zero();
				Vector3r avg_angular = Vector3r::Zero();
				if (!body.isGrounded()){
					avg_linear = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
					avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
					const Wrench drag_wrench = getDragWrench(body, current.pose.orientation, avg_linear, avg_angular);
					next_wrench = body_wrench + drag_wrench;
					next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
				}
				if (body.isGrounded() && (normalizedForce >= normalizedGravity))
				{
					body.setGrounded(false);
				}
				if (!body.isGrounded()) {
					const Vector3r angular_momentum = body.getInertia() * avg_angular;
					const Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
					next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;
					next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
					next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);
					if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.linear = Vector3r::Zero();
					}
					if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.angular = Vector3r::Zero();
					}
				}
				computeNextPose(dt, current.pose, avg_linear, avg_angular, next);
			}
			static void getNextKinematicsNoCollision(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
				Kinematics::State& next, Wrench& next_wrench, LogFileWriter & logger)
			{
				const real_T dt_real = static_cast<real_T>(dt);

				Vector3r avg_linear = Vector3r::Zero();
				Vector3r avg_angular = Vector3r::Zero();

				/************************* Get force and torque acting on body ************************/
				//set wrench sum to zero
				const Wrench body_wrench = getBodyWrench(body, current.pose.orientation, logger);

				if (body.isGrounded()) {
					
						
					// make it stick to the ground until we see body wrench force greater than gravity.
					float normalizedForce = body_wrench.force.squaredNorm();
					float normalizedGravity = body.getEnvironment().getState().gravity.squaredNorm();
					Vector3r FrictionForce = body.getEnvironment().getState().gravity * 0.2f;
					float normalizedFricForce = FrictionForce.squaredNorm();
					if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
					{
						logger.write("__________________________________\n is grounded");
						logger.write(normalizedForce);
						logger.write("(");
						logger.write(body_wrench.force);
						logger.write(")");
						logger.write("more than");
						logger.write(normalizedFricForce);
						logger.write("|");
						logger.write( body.getEnvironment().getState().geo_point.altitude);
						logger.write("|\n__________________________________\n");
					}
					if (normalizedForce >= normalizedGravity)
					{
						//throttledLogOutput("*** Losing ground lock due to body_wrench " + VectorMath::toString(body_wrench.force), 0.1);
						body.setGrounded(false);
					}
					//next_wrench.force = Vector3r::Zero();
					next_wrench = body_wrench;
					/*next_wrench.force << body_wrench.force.x(), body_wrench.force.x()*/
					next_wrench.torque = Vector3r::Zero();
					next.accelerations.linear = Vector3r::Zero();
				}
				else {
					if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
						logger.write("NOT grounded\n");
					//add linear drag due to velocity we had since last dt seconds
					//drag vector magnitude is proportional to v^2, direction opposite of velocity
					//total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
					//To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
					avg_linear = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
					avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
					const Wrench drag_wrench = getDragWrench(body, current.pose.orientation, avg_linear, avg_angular);

					next_wrench = body_wrench + drag_wrench;

					//Utils::log(Utils::stringf("B-WRN %s: ", VectorMath::toString(body_wrench.force).c_str()));
					//Utils::log(Utils::stringf("D-WRN %s: ", VectorMath::toString(drag_wrench.force).c_str()));

					/************************* Update accelerations due to force and torque ************************/
					//get new acceleration due to force - we'll use this acceleration in next time step

					next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
				}


				if (body.isGrounded()) {
					// this stops vehicle from vibrating while it is on the ground doing nothing.
					next.accelerations.angular = Vector3r::Zero();
					next.twist.linear = Vector3r::Zero();
					next.twist.angular = Vector3r::Zero();
					logger.write("STOP vehicle from VIBRATION\n");
				}
				else {
					//get new angular acceleration
					//Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
					//we will use torque to find out the angular acceleration
					//angular momentum L = I * omega
					const Vector3r angular_momentum = body.getInertia() * avg_angular;
					const Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
					//new angular acceleration - we'll use this acceleration in next time step
					next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;

					/************************* Update pose and twist after dt ************************/
					//Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
					logger.write("UPDATE vehicle pose\n");
					next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
					next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);

					//if controller has bug, velocities can increase idenfinitely 
					//so we need to clip this or everything will turn in to infinity/nans

					if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.linear = Vector3r::Zero();
					}
					//
					//for disc of 1m radius which angular velocity translates to speed of light on tangent?
					if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.angular = Vector3r::Zero();
					}
				}

				computeNextPose(dt, current.pose, avg_linear, avg_angular, next);

				//Utils::log(Utils::stringf("N-VEL %s %f: ", VectorMath::toString(next.twist.linear).c_str(), dt));
				//Utils::log(Utils::stringf("N-POS %s %f: ", VectorMath::toString(next.pose.position).c_str(), dt));

			}

			static void computeNextPose(TTimeDelta dt, const Pose& current_pose, const Vector3r& avg_linear, const Vector3r& avg_angular, Kinematics::State& next)
			{
				real_T dt_real = static_cast<real_T>(dt);

				next.pose.position = current_pose.position + avg_linear * dt_real;

				//use angular velocty in body frame to calculate angular displacement in last dt seconds
				real_T angle_per_unit = avg_angular.norm();
				if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
					//convert change in angle to unit quaternion
					AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
					Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
					/*
					Add change in angle to previous orientation.
					Proof that this is q0 * q1:
					If rotated vector is qx*v*qx' then qx is attitude
					Initially we have q0*v*q0'
					Lets transform this to body coordinates to get
					q0'*(q0*v*q0')*q0
					Then apply q1 rotation on it to get
					q1(q0'*(q0*v*q0')*q0)q1'
					Then transform back to world coordinate
					q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
					which simplifies to
					q0(q1(v)q1')q0'
					Thus new attitude is q0q1
					*/
					next.pose.orientation = current_pose.orientation * angle_dt_q;
					if (VectorMath::hasNan(next.pose.orientation)) {
						//Utils::DebugBreak();
						Utils::log("orientation had NaN!", Utils::kLogLevelError);
					}

					//re-normalize quaternion to avoid accumulating error
					next.pose.orientation.normalize();
				}
				else //no change in angle, because angular velocity is zero (normalized vector is undefined)
					next.pose.orientation = current_pose.orientation;
			}

		private:
			static constexpr uint kCollisionResponseCycles = 1;
			static constexpr float kAxisTolerance = 0.25f;
			static constexpr float kRestingVelocityMax = 0.1f;
			static constexpr float kDragMinVelocity = 0.1f;

			std::stringstream debug_string_;
			bool enable_ground_lock_;
			TTimePoint last_message_time;

			/*Логирование*/
			std::string plane_logger_name_ = "";
			LogFileWriter plane_logger_;
			//static uint isFullLogging_ = 1;
		};

	}
} //namespace
#endif


