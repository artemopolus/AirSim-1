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
#include "common/Settings.hpp" 
/* добавляем логирование*/
#include "common/LogFileWriter.hpp"
#include "vehicles/plane/Plane.hpp"

#define LOG_WRITE logger.write

namespace msr {
	namespace airlib {

		class ExPhysicsEngine : public PhysicsEngineBase {
		public:
			ExPhysicsEngine(bool enable_ground_lock = true)
				: enable_ground_lock_(enable_ground_lock)
			{
				//std::string filename = std::string("J:/Unreal/LogAirSim/") + plane_logger_name_ + std::string("_PhysicsEngine.txt");
				std::string filename = Settings::getUserDirectoryFullPath(plane_logger_name_ + std::string("_PhysicsEngine.txt"));
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
			static void calcDebugBodyKinematicsNoCollisions(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
				Kinematics::State& next, Wrench& next_wrench, LogFileWriter & logger)
			{
				getNextKinematicsNoCollision2(dt, body, current, next, next_wrench, logger);
			}
			static void calcDebugBodyKinematicsOnCollisions(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
				const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock, LogFileWriter & logger)
			{
				getNextKinematicsOnCollision2(dt, collision_info, body, current,
					next, next_wrench, enable_ground_lock, logger, true);
			}
			static Wrench calcDebugGetBodyWrench(PhysicsBody & body, Quaternionr & orientation, LogFileWriter & logger)
			{
				 return getBodyWrench(body, orientation, logger);
			}
			static void stateReport2log(Kinematics::State state, LogFileWriter & logger)
			{
				LOG_WRITE("\nReport state");
				LOG_WRITE("\nPostion:");
				LOG_WRITE(state.pose.position);
				LOG_WRITE("\nOrientation:");
				LOG_WRITE(state.pose.orientation);
				real_T pitch, roll, yaw;
				VectorMath::toEulerianAngle(state.pose.orientation, pitch, roll, yaw);
				float unit = 180.0f / (float)M_PI;
				pitch *= unit; roll *= unit; yaw *= unit;
				LOG_WRITE("\nEuler:");
				LOG_WRITE(pitch); LOG_WRITE(roll); LOG_WRITE(yaw);
				LOG_WRITE("\nLinear velocity:");
				LOG_WRITE(state.twist.linear);
				LOG_WRITE("\nLinear acceleration:");
				LOG_WRITE(state.accelerations.linear);
				LOG_WRITE("\nAngular velocity:");
				LOG_WRITE(state.twist.angular);
				LOG_WRITE("\nAngular acceleartions:");
				LOG_WRITE(state.accelerations.angular);
				logger.endl();
			}
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
				/*plane_logger_.write("ORIENTATION \t >>>> [");
				plane_logger_.write(current.pose.orientation);
				plane_logger_.write("]\n");*/
				plane_logger_.write("=======================\n[NEW_UPD]");
				plane_logger_.write("s");
				plane_logger_.write(current.pose.position);
				plane_logger_.write("v");
				plane_logger_.write(current.twist.linear);
				plane_logger_.write("a");
				plane_logger_.write(current.accelerations.linear);
				plane_logger_.write("w");
				plane_logger_.write(current.twist.angular);
				plane_logger_.write("t");
				plane_logger_.write(current.accelerations.angular);
				plane_logger_.write("q");
				plane_logger_.write(current.pose.orientation);
				plane_logger_.write("\n");
				/*
				plane_logger_.write("ACC  ANG \t >>>> [");
				plane_logger_.write(current.accelerations.angular);
				plane_logger_.write("]\n");*/
				next_wrench = getBodyWrench(body, current.pose.orientation, plane_logger_);

				plane_logger_.write("[NEXT_WRENCH]");
				plane_logger_.write("force");
				plane_logger_.write(next_wrench.force);
				plane_logger_.write("torque");
				plane_logger_.write(next_wrench.torque);
				plane_logger_.endl();
				//first compute the response as if there was no collision
				//this is necessary to take in to account forces and torques generated by body
				//getNextKinematicsNoCollision(dt, body, current, next, next_wrench, plane_logger_);

				//if there is collision, see if we need collision response
				const CollisionInfo collision_info = body.getCollisionInfo();
				plane_logger_.write("collision info:\nobj_id:");
				plane_logger_.write(collision_info.object_id);
				plane_logger_.write("obj_name:");
				plane_logger_.write(collision_info.object_name);
				plane_logger_.write("time_stmp");
				plane_logger_.write(collision_info.time_stamp);
				CollisionResponse& collision_response = body.getCollisionResponseInfo();
				plane_logger_.write("===============[COLLISION INFO]:");
				//FString info_text = TEXT("");
				if (collision_info.has_collided || body.isGrounded()) {
					plane_logger_.write("HAS COLLIDED\n");
					//info_text += TEXT("COLLIDED");
					bool is_collision_response = getNextKinematicsOnCollision2(dt, collision_info, body, current,
						next, next_wrench, enable_ground_lock_, plane_logger_, false);
					plane_logger_.write("================UPDATE COLLISION RESPONSE:");
					if (collision_response.collision_time_stamp != collision_info.time_stamp) {
						plane_logger_.write("YES\n");
						updateCollisionResponseInfo(collision_info, next, is_collision_response, collision_response);
					}
					else {
						plane_logger_.write("NO\n");
					}
				}
				else {
					//info_text += TEXT("FREE");
					plane_logger_.write("NO\n");
					getNextKinematicsNoCollision2(dt, body, current, next, next_wrench, plane_logger_);
				}

				//UAirBlueprintLib::LogMessage(TEXT("[COLLISSION INFO]:"), info_text, LogDebugLevel::Informational);
				
				//if collision was already responded then do not respond to it until we get updated information
				//if (body.isGrounded() || (collision_info.has_collided && collision_response.collision_time_stamp != collision_info.time_stamp)) {
				//	plane_logger_.write("Get ON Collision\n");
				//	bool is_collision_response = getNextKinematicsOnCollision(dt, collision_info, body,
				//		current, next, next_wrench, enable_ground_lock_, plane_logger_);
				//	updateCollisionResponseInfo(collision_info, next, is_collision_response, collision_response);
				//	//throttledLogOutput("*** has collision", 0.1);
				//}
				//else throttledLogOutput("*** no collision", 0.1);

				//Utils::log(Utils::stringf("T-VEL %s %" PRIu64 ": ", 
				//    VectorMath::toString(next.twist.linear).c_str(), clock()->getStepCount()));

				body.setWrench(next_wrench);
				plane_logger_.write("[UPDATE KINEMATICS]");
				plane_logger_.write("s");
				plane_logger_.write(next.pose.position);
				plane_logger_.write(next.pose.orientation);
				plane_logger_.write("v");
				plane_logger_.write(next.twist.linear);
				plane_logger_.write("a");
				plane_logger_.write(next.accelerations.linear);
				plane_logger_.write("w");
				plane_logger_.write(next.twist.angular);
				plane_logger_.write("t");
				plane_logger_.write(next.accelerations.angular);
				plane_logger_.write("\n========================\n==========END UPDATE");
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
					logger.write("[PLANE_WRENCH]");

				//calculate total force on rigid body's center of gravity
				for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
					//aggregate total
					const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
					const auto& vertex_wrench = vertex.getWrench();
					wrench += vertex_wrench;
					if (vertex.getObjType() == UpdatableObject::typeUpdObj::rotor) {
						logger.write("rotor");
					}
					else if (vertex.getObjType() == UpdatableObject::typeUpdObj::rudder) {
						logger.write("rudder");
					}
					else if (vertex.getObjType() == UpdatableObject::typeUpdObj::wing) {
						logger.write("wing");
					}
					else {
						logger.write("unknown");
					}
					logger.write("f");
					logger.write(vertex_wrench.force);
					logger.write("t");
					logger.write(vertex_wrench.torque);
					logger.write("p");
					logger.write(vertex.getPosition());
					//add additional torque due to force applies farther than COG
					// tau = r X F
					wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
					
				}
				if (body.getObjType() == UpdatableObject::typeUpdObj::plane) {
					logger.write("[SUM]\tf");
					logger.write(wrench.force);
					logger.write("t");
					logger.write(wrench.torque);
				}
				//convert force to world frame, leave torque to local frame
				wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);
				if (body.getObjType() == UpdatableObject::typeUpdObj::plane)
				{
					logger.write("[SUM_W]\tf");
					logger.write(wrench.force);
					logger.write("t");
					logger.write(wrench.torque);
					logger.endl();
				}
				return wrench;
			}
			static void resForceFromFriction(float force_main, float force_fric, real_T mass, real_T dt,
				float & acc, float & velocity, LogFileWriter & logger)
			{
				logger.write("[inp frc]\tf_m");
				logger.write(force_main);
				logger.write("f_fr");
				logger.write(force_fric);
				logger.write("v");
				logger.write(velocity);
				logger.write("a");
				logger.write(acc);
				logger.endl();
				float prev_acc = acc;
				if (velocity == 0) {
					if (force_main > 0) {
						float diff = (force_main - force_fric);
						if (diff > 0) {
							acc = diff / mass;}
						else {
							acc = 0;}
					}
					else {
						float diff = (force_main + force_fric);
						if (diff < 0) {
							acc = diff / mass;
						}
						else {
							acc = 0;}
					}
				}
				else if (velocity > 0) { // ускорение прекращающее движение в следующий тик
					float diff = (force_main - force_fric);
					if ((velocity + diff / mass * dt) < 0) {
						acc = 0; velocity = 0;	prev_acc = 0;					}
					else {
						acc = diff / mass;}
				}
				else {
					float diff = (force_main + force_fric);
					if ((velocity + diff / mass * dt) > 0) {
						acc = 0; velocity = 0;	prev_acc = 0;					}
					else {
						acc = diff / mass;}
				}
				velocity += 0.5f*(acc + prev_acc) * dt;

				logger.write("[out frc]\ta");
				logger.write(acc);
				logger.write("v");
				logger.write(velocity);
				logger.endl();
			}
			static void resMomentumFromFriction(const Vector3r m_main, const Vector3r m_fric, const Vector3r m_react,
				const Matrix3x3r InertInv, const real_T dt,
				Vector3r & acc_ang, Vector3r & vel_ang, LogFileWriter & logger)
			{
				float f_x = std::abs(m_fric.x()) + m_react.x(), f_y = std::abs(m_fric.y()) + m_react.y(), f_z = std::abs(m_fric.z()) + m_react.z();
				float value_v = 0.00005f;
				float value_f = 0.00005f;
				uint velocity_is_zero = ((std::abs(vel_ang.x()) < value_v)
					&& (std::abs(vel_ang.y()) < value_v)
					&& (std::abs(vel_ang.z()) < value_v)) ? 1 : 0;
				uint force_is_zero = ((std::abs(m_main.x()) < value_f)
					&& (std::abs(m_main.y()) < value_f)
					&& (std::abs(m_main.z()) < value_f)) ? 1 : 0;
				logger.write(">>>> result Moment From Friction<<<\n[INPUT]m_main");
				logger.write(m_main);
				logger.write("m_fric");
				logger.write(m_fric);
				logger.write("m_react");
				logger.write(m_react);

				if (velocity_is_zero && force_is_zero) {
					vel_ang = Vector3r::Zero();
					acc_ang = Vector3r::Zero();
					return;
				}
				if (velocity_is_zero) {
					f_x = (m_main.x() > 0) ? -f_x : f_x;
					f_y = (m_main.y() > 0) ? -f_y : f_y;
					f_z = (m_main.y() > 0) ? -f_z : f_z;
					Vector3r acc_ang_t = InertInv * (m_main + Vector3r(f_x, f_y, f_z));
					Vector3r vel_ang_t = acc_ang_t * dt;
					float vel_x = vel_ang_t.x(), vel_y = vel_ang_t.y(), vel_z = vel_ang_t.z();
					float acc_x = acc_ang_t.x(), acc_y = acc_ang_t.y(), acc_z = acc_ang_t.z();
					if (vel_ang_t.x()*m_main.x() <= 0) {
						vel_x = 0; acc_x = 0;
					}
					if (vel_ang_t.y()*m_main.y() <= 0) {
						vel_y = 0; acc_y = 0;
					}
					if (vel_ang_t.z()*m_main.z() <= 0) {
						vel_z = 0; acc_z = 0;
					}
					vel_ang = Vector3r(vel_x, vel_y, vel_z);
					acc_ang = Vector3r(acc_x, acc_y, acc_z);
				}
				else {
					f_x = (vel_ang.x() > 0) ? -f_x : f_x;
					f_y = (vel_ang.y() > 0) ? -f_y : f_y;
					f_z = (vel_ang.y() > 0) ? -f_z : f_z;
					Vector3r acc_ang_t = InertInv * (m_main + Vector3r(f_x, f_y, f_z));
					Vector3r vel_ang_t = acc_ang_t * dt;
					float vel_x = vel_ang_t.x(), vel_y = vel_ang_t.y(), vel_z = vel_ang_t.z();
					float acc_x = acc_ang_t.x(), acc_y = acc_ang_t.y(), acc_z = acc_ang_t.z();
					if (vel_ang_t.x()*vel_ang.x() <= 0) {
						vel_x = 0; acc_x = vel_ang.x() / dt;
					}
					if (vel_ang_t.y()*vel_ang.y() <= 0) {
						vel_y = 0; acc_y = vel_ang.y() / dt;
					}
					if (vel_ang_t.z()*vel_ang.z() <= 0) {
						vel_z = 0; acc_z = vel_ang.z() / dt;
					}
					vel_ang = Vector3r(vel_x, vel_y, vel_z);
					acc_ang = Vector3r(acc_x, acc_y, acc_z);
				}
				logger.write("friction plus reation ");
				logger.write(Vector3r(f_x, f_y, f_z));
				logger.write("\n[OUTPUT]vel_ang");
				logger.write(vel_ang);
				logger.write("acc_ang");
				logger.write(acc_ang);
				logger.endl();
			}
			static bool getNextKinematicsOnCollision2(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
				const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock, LogFileWriter & logger, const bool log_on)
			{
				const bool is_horizontal_ground = Utils::isApproximatelyEqual(std::abs(collision_info.normal.z()), 1.0f, kAxisTolerance);

				const real_T dt_real = static_cast<real_T>(dt);
				const Wrench body_wrench = next_wrench;
				if (log_on) {
					logger.write("=================================================\n");
					logger.write("[OC_INPUT]\tf");
					logger.write(body_wrench.force);
					logger.write("t");
					logger.write(body_wrench.torque);
					logger.write("\nvel lin");
					logger.write(current.twist.linear);
					logger.write("acc");
					logger.write(current.accelerations.linear);
					logger.write("\nvel ang");
					logger.write(current.twist.angular);
					logger.write("acc");
					logger.write(current.accelerations.angular);
					logger.write("\n[collision] impact point");
					logger.write(collision_info.impact_point);
					logger.write("position");
					logger.write(collision_info.position);
					logger.write("normal");
					logger.write(collision_info.normal);
					logger.write("penetr depth");
					logger.write(collision_info.penetration_depth);
					logger.write("dt");
					logger.write(dt_real);
					logger.endl();
				}

				//рычаг коллизии к центру масс
				Vector3r r = collision_info.impact_point - collision_info.position;
				//пересчитываем все силы и моменты: приложенные силы к объекту, реакции опоры, трение
				const Vector3r norm_world = Vector3r(0, 0, 1);
				const Eigen::Quaternionf q2collision = VectorMath::toQuaternion(norm_world, collision_info.normal);
				Vector3r gravity_w = body.getEnvironment().getState().gravity;
				Vector3r sum_force_world = body_wrench.force
					//+ body_wrench.torque.cross(r)
					+ gravity_w * body.getMass();
				Vector3r sum_f_c = VectorMath::transformToBodyFrame(sum_force_world, q2collision); // _f_c == _force_collision
				Vector3r vel_c = VectorMath::transformToBodyFrame(current.twist.linear, q2collision);
				Vector3r acc_c = VectorMath::transformToBodyFrame(current.accelerations.linear, q2collision);
				const Vector3r ang_c = VectorMath::transformToBodyFrame(current.twist.angular, q2collision);
				const Vector3r tau_c = VectorMath::transformToBodyFrame(current.accelerations.angular, q2collision);

				Vector3r r_c = VectorMath::transformToBodyFrame(r, q2collision);
				Vector3r r_inv_c = -r_c;
				const Vector3r torq_c = VectorMath::transformToBodyFrame(body_wrench.torque, q2collision);
				Vector3r hitnorm_c = VectorMath::transformToBodyFrame(collision_info.normal, q2collision);
				Vector3r gravity_c = VectorMath::transformToBodyFrame(gravity_w, q2collision);

				if (log_on)
				{
				logger.write("[OC_BODY]\tf");
				logger.write(sum_f_c);
				logger.write("v");
				logger.write(vel_c);
				logger.write("a");
				logger.write(acc_c);
				logger.endl();

				}
				float reaction = (sum_f_c.z() > 0) ? 0 : (-sum_f_c.z());
				float mu = 0.0005f;
				float frc_friction = mu * reaction;
				
				float x_f_c = acc_c.x(), y_f_c = acc_c.y(), x_v_c = vel_c.x(), y_v_c = vel_c.y();
				resForceFromFriction(sum_f_c.x(), frc_friction, body.getMass(), dt_real, x_f_c, x_v_c, logger);
				resForceFromFriction(sum_f_c.y(), frc_friction, body.getMass(), dt_real, y_f_c, y_v_c, logger);
				float z_f_c = (sum_f_c.z() + reaction);
				Vector3r res_a_lin_c = Vector3r(x_f_c, y_f_c, 0);

				const float eps = 0.2f;
				const float vertical_velocity_collision = collision_info.normal.dot(current.twist.linear);
				const float eqzerovelocity = 0.1f;
				float vert_vel_z = vel_c.z() + (z_f_c + acc_c.z())*0.5f*dt_real;
				if (log_on) 
				{
					logger.write("===================collision type: ");
					logger.write(vertical_velocity_collision);
					logger.write("vert acc z");
					logger.write(z_f_c);
					logger.write("vert vel z");
					logger.write(vert_vel_z);
				}
				if ((vert_vel_z > -eqzerovelocity)&&(vert_vel_z < eqzerovelocity)) {
					if(log_on)
						logger.write("stay ground \r\n");
					vert_vel_z = 0.0f;
				}
				else if(vert_vel_z >= eqzerovelocity ) {
					if(log_on)
						logger.write(" out collision \r\n");
				}
				else {
					if(log_on)
						logger.write(" penetrate \r\n");
					vert_vel_z = -vel_c.z()*eps;
				}			
				//float vert_vel_z = (collission_type == 2) ? -vel_c.z()*eps : vel_c.z();

				Vector3r res_v_lin_c = Vector3r(x_v_c, y_v_c, vert_vel_z);

				if (log_on) {
					logger.write("[OC_CALC_SUM_FORCES]\tF_w");
					logger.write(sum_force_world);
					logger.write("F_c");
					logger.write(sum_f_c);
					logger.write("F_r");
					logger.write(frc_friction);
					logger.write("a");
					logger.write(res_a_lin_c);
					logger.write("v");
					logger.write(res_v_lin_c);
					logger.endl();
				}
				Vector3r reaction_c = Vector3r(0, 0, reaction);
				Vector3r sum_m_c = reaction_c.cross(r_c) + torq_c;
				Vector3r m_friction = Vector3r(frc_friction, frc_friction, 0).cross(r_c);
				//float x_v_ang_c = ang_c.x(), y_v_ang_c = ang_c.y();
				Vector3r res_a_ang_c;
				Vector3r res_v_ang_c = ang_c;
				Vector3r m_reaction = body.getReaction(r_inv_c, hitnorm_c, mu, gravity_c);
				if (log_on)
				{
					logger.write("============CALC MOMENTUM\nr_inv_c");
					logger.write(r_inv_c);
					logger.write("[hitnorm_c] ");
					logger.write(hitnorm_c);
					logger.write("[gravity_c] ");
					logger.write(gravity_c);
				}
				resMomentumFromFriction(sum_m_c, m_friction, m_reaction,
					body.getInertiaInv(), dt_real,
					res_a_ang_c, res_v_ang_c, logger);
				
				res_a_ang_c = Vector3r::Zero();
				res_v_ang_c = Vector3r::Zero();


				Vector3r avg_angular = Vector3r::Zero();
				const Quaternionr body_orientation = body.getPose().orientation;
				//const Vector3r angular_vel_b = VectorMath::transformToBodyFrame(current.twist.angular, body.getPose().orientation);
				//const Vector3r angular_acc_b = VectorMath::transformToBodyFrame(current.accelerations.angular, body.getPose().orientation);
				//const Vector3r torque_b = VectorMath::transformToBodyFrame(next_wrench.torque, body.getPose().orientation);
				const Vector3r angular_vel_b = current.twist.angular;
				const Vector3r angular_acc_b = current.accelerations.angular;
				const Vector3r torque_b = next_wrench.torque;

				avg_angular = angular_vel_b + angular_acc_b * dt_real;
				Matrix3x3r I = body.getInertia();
				const Vector3r angular_momentum = I * avg_angular;
				//const Vector3r angular_momentum_rate = torque_b - avg_angular.cross(angular_momentum);
				const Vector3r angular_momentum_rate = torque_b;
				//Matrix3x3r IInv = body.getInertiaInv();
				Matrix3x3r IInv = Matrix3x3r::Zero();
				IInv(0, 0) = 1;
				IInv(1, 1) = 1;
				IInv(2, 2) = 1;
				res_a_ang_c = IInv * angular_momentum_rate;
				res_v_ang_c = angular_vel_b + (angular_acc_b + res_a_ang_c) * (0.5f * dt_real);
				if (log_on)
				{
					LOG_WRITE("=>Moment => angular velocity\nangular_velocity:");
					LOG_WRITE(avg_angular);
					LOG_WRITE("\nangular_momentum");
					LOG_WRITE(angular_momentum);
					LOG_WRITE("\nangular_momentum_rate");
					LOG_WRITE(angular_momentum_rate);
					LOG_WRITE("\nI\n");
					LOG_WRITE(I);
					LOG_WRITE("\nI_inv\n");
					LOG_WRITE(IInv);
					LOG_WRITE("\nT");
					LOG_WRITE(torque_b);
					LOG_WRITE("\ninit_acc");
					LOG_WRITE(angular_acc_b);
					LOG_WRITE("\nint_vel");
					LOG_WRITE(angular_vel_b);

					LOG_WRITE("\nacc_ang");
					LOG_WRITE(res_a_ang_c);
					LOG_WRITE("\nvel_ang");
					LOG_WRITE(res_v_ang_c);

					logger.endl();
				}

				next.twist.linear = VectorMath::transformToWorldFrame(res_v_lin_c, q2collision);
				next.accelerations.linear = VectorMath::transformToWorldFrame(res_a_lin_c, q2collision);
				//next.twist.angular = VectorMath::transformToWorldFrame(res_v_ang_c, body_orientation);
				//next.accelerations.angular = VectorMath::transformToWorldFrame(res_a_ang_c, body_orientation);
				next.twist.angular = res_v_ang_c;
				next.accelerations.angular = res_a_ang_c;

				if (log_on) {
					logger.write("=============CALC IMPULSE\nBODY[position]");
					logger.write(body.getPose().position);
					real_T pitch, roll, yaw;
					VectorMath::toEulerianAngle(body.getPose().orientation, pitch, roll, yaw);
					logger.write("\n[orientation]");
					float unit = 180.0f / (float)M_PI;
					logger.write(Vector3r(pitch*unit, roll*unit, yaw*unit));
					Vector3r norm = Vector3r(0, 0, 1);
					norm = VectorMath::transformToBodyFrame(norm, body.getPose().orientation);
					logger.write("\n[norm]");
					logger.write(norm);
					logger.write("\n[COLLISION projection]hitnormal");
					logger.write(hitnorm_c);
					logger.write("r_inv");
					logger.write(r_inv_c);
					logger.write("g_c");
					logger.write(gravity_c);
					logger.write("r");
					logger.write(r);
					logger.write("r_c");
					logger.write(r_c);
					logger.write("\n[MOMENTUM] SUM");
					logger.write(sum_m_c);
					logger.write("FRIC");
					logger.write(m_friction);
					logger.write("REAC");
					logger.write(m_reaction);
					logger.write("\n[OC_OUTPUT]\tv");
					logger.write(next.twist.linear);
					logger.write("a");
					logger.write(next.accelerations.linear);
					logger.write("w");
					logger.write(next.twist.angular);
					logger.write("t");
					logger.write(next.accelerations.angular);
					logger.endl();
				}
				/*FString info_text = TEXT("col_pos[")
					+ FString::SanitizeFloat(collision_info.position.z())
					+TEXT("]penetr[")
					+FString::SanitizeFloat(collision_info.penetration_depth)
					+TEXT("]out_vel[")
					+FString::SanitizeFloat(res_v_lin_c.x())
					+TEXT(",")
					+FString::SanitizeFloat(res_v_lin_c.y())
					+TEXT(",")
					+FString::SanitizeFloat(res_v_lin_c.z())
					+TEXT("]pos[")
					+FString::SanitizeFloat(current.pose.position.z())
					+TEXT("]\nvel[")
					+FString::SanitizeFloat(next.twist.linear.x())
					+TEXT(",")
					+FString::SanitizeFloat(next.twist.linear.y())
					+TEXT(",")
					+FString::SanitizeFloat(next.twist.linear.z())
					+TEXT("]\ngr[")
					+ (body.isGrounded()? TEXT("GROUNDED") : TEXT(""))
					+TEXT("]")
					;*/

				computeNextPose(dt, current.pose, next.twist.linear, next.twist.angular, next);

				if ((res_v_lin_c.z() == 0.0) && (!body.isGrounded()))
				{
					body.setGrounded(true);
				next.pose.position = collision_info.position + collision_info.normal*1.0f;
				}
				if ((res_v_lin_c.z() != 0.0) && (body.isGrounded()))
				{
					body.setGrounded(false);
				}
				//UAirBlueprintLib::LogMessage(TEXT("On collision:"), info_text, LogDebugLevel::Informational);

				return true;
			}
			static void getNextKinematicsNoCollision2(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
				Kinematics::State& next, Wrench& next_wrench, LogFileWriter & logger)
			{
				const real_T dt_real = static_cast<real_T>(dt);
				const Wrench body_wrench = next_wrench;
				logger.write("[NC_INPUT]\tF");
					logger.write(body_wrench.force);
					logger.write("M");
					logger.write(body_wrench.torque);
					logger.write("v");
					logger.write(current.twist.linear);
					logger.write("a");
					logger.write(current.accelerations.linear);
					logger.write("w");
					logger.write(current.twist.angular);
					logger.write("t");
					logger.write(current.accelerations.angular);
					float normalizedForce = body_wrench.force.squaredNorm();
				float normalizedGravity = body.getEnvironment().getState().gravity.squaredNorm();
				Vector3r avg_linear = Vector3r::Zero();
				Vector3r avg_angular = Vector3r::Zero();
				logger.write("\n1step acc calc: DF");
				if (!body.isGrounded()) {
					avg_linear = current.twist.linear + current.accelerations.linear * dt_real;
					avg_angular = current.twist.angular + current.accelerations.angular * dt_real;
					const Wrench drag_wrench = getDragWrench(body, current.pose.orientation, avg_linear, avg_angular);
					next_wrench = body_wrench + drag_wrench;
					logger.write(drag_wrench.force);
					next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
				}
				logger.write("F");
				logger.write(next_wrench.force);
				logger.write("a");
				logger.write(next.accelerations.linear);
				logger.write("v");
				logger.write(avg_linear);
				logger.write("w");
				logger.write(avg_angular);
		
				if (body.isGrounded() && (normalizedForce >= normalizedGravity))
				{
					body.setGrounded(false);
				}
				if (!body.isGrounded()) {
					logger.write("\n2step v calc: t");
					const Vector3r angular_momentum = body.getInertia() * avg_angular;
					const Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
					next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;
					next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
					next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);
					logger.write("v");
					logger.write(next.twist.linear);
					logger.write("w");
					logger.write(next.twist.angular);
					logger.write("t");
					logger.write(next.accelerations.angular);
					logger.endl();
					if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.linear = Vector3r::Zero();
					}
					if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
						next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
						next.accelerations.angular = Vector3r::Zero();
					}
				}
				logger.write("[NC_OUTPUT]");
				logger.write("v");
				logger.write(avg_linear);
				logger.write("w");
				logger.write(avg_angular);
				logger.endl();

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
						logger.write(body.getEnvironment().getState().geo_point.altitude);
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


