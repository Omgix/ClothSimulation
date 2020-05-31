#ifndef CAMERA_H
#define CAMERA_H

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective

enum Movement {
	FORWARD,
	BACKWARD,
	LEFT, 
	RIGHT,
	UP,
	DOWN,
	TURNLEFT,
	TURNRIGHT,
	TURNUP,
	TURNDOWN
};

const float YAW =			90.f;
const float PITCH =			0.0f;
const float FOV =			45.0f;
const float SENSITIVITY =	10.0f;
const float SPEED =			2.0f;

class Camera {
private:
	float _yaw;
	float _pitch;
	float _fov;
	float _sensitivity;
	float _speed;

	glm::vec3 _world_up;
	glm::vec3 _position;
	glm::vec3 _front;
	glm::vec3 _right;
	glm::vec3 _up;

	void update_camera_vectors() {
		_front.x = glm::cos(glm::radians(_pitch)) * glm::cos(glm::radians(_yaw));
		_front.y = glm::sin(glm::radians(_pitch));
		_front.z = glm::cos(glm::radians(_pitch)) * glm::sin(glm::radians(_yaw));
		_front = glm::normalize(_front);

		_right = glm::normalize(glm::cross(_front, _world_up));
		_up = glm::normalize(glm::cross(_right, _front));
	}
public:
	explicit Camera(glm::vec3 &position = glm::vec3(0, 0, 0), glm::vec3 &up = glm::vec3(0, 1, 0), float yaw = YAW, float pitch = PITCH):
		_yaw(yaw), _pitch(pitch), _fov(FOV), _sensitivity(SENSITIVITY), _speed(SPEED), _world_up(up), _position(position),
	    _up(up)
	{ 
		update_camera_vectors();
	}
	explicit Camera(float posX = 0, float posY = 0, float posZ = 0, float upX = 0, float upY = 1, float upZ = 0, float yaw = YAW, float pitch = PITCH):
		_yaw(yaw), _pitch(pitch), _fov(FOV), _sensitivity(SENSITIVITY), _speed(SPEED), _world_up(glm::vec3(upX, upY, upZ)), 
		_position(glm::vec3(posX, posY, posZ)), _up(glm::vec3(upX, upY, upZ))
	{
		update_camera_vectors();
	}
	glm::mat4 view_matrix() {
		return glm::lookAt(_position, _position + _front, _up);
	}
	void process_keyboard(Movement direction, float deltaTime)
	{
		float velocity = _speed * deltaTime;
		if (direction == FORWARD)
			_position += _front * velocity;
		else if (direction == BACKWARD)
			_position -= _front * velocity;
		else if (direction == LEFT)
			_position -= _right * velocity;
		else if (direction == RIGHT)
			_position += _right * velocity;
		else if (direction == UP)
			_position += _up * velocity;
		else if (direction == DOWN)
			_position -= _up * velocity;
		else if (direction == TURNLEFT)
			_yaw -= velocity * _sensitivity;
		else if (direction == TURNRIGHT)
			_yaw += velocity * _sensitivity;
		else if (direction == TURNUP)
			_pitch += velocity * _sensitivity;
		else if (direction == TURNDOWN)
			_pitch -= velocity * _sensitivity;

		update_camera_vectors();
	}
	void process_mouse_scroll(float offset)
	{
		_fov = glm::max(1.0f, glm::min(60.0f, _fov - offset));
	}
	const glm::vec3& position() {
		return _position;
	}
	float fov() {
		return _fov;
	}
	const glm::vec3& front() {
		return _front;
	}
	const glm::vec3& up() {
		return _up;
	}
};

#endif
