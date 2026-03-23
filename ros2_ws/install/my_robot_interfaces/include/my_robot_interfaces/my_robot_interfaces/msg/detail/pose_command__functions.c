// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_robot_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice
#include "my_robot_interfaces/msg/detail/pose_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_robot_interfaces__msg__PoseCommand__init(my_robot_interfaces__msg__PoseCommand * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  // cartesian_path
  return true;
}

void
my_robot_interfaces__msg__PoseCommand__fini(my_robot_interfaces__msg__PoseCommand * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  // cartesian_path
}

bool
my_robot_interfaces__msg__PoseCommand__are_equal(const my_robot_interfaces__msg__PoseCommand * lhs, const my_robot_interfaces__msg__PoseCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // cartesian_path
  if (lhs->cartesian_path != rhs->cartesian_path) {
    return false;
  }
  return true;
}

bool
my_robot_interfaces__msg__PoseCommand__copy(
  const my_robot_interfaces__msg__PoseCommand * input,
  my_robot_interfaces__msg__PoseCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // cartesian_path
  output->cartesian_path = input->cartesian_path;
  return true;
}

my_robot_interfaces__msg__PoseCommand *
my_robot_interfaces__msg__PoseCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_interfaces__msg__PoseCommand * msg = (my_robot_interfaces__msg__PoseCommand *)allocator.allocate(sizeof(my_robot_interfaces__msg__PoseCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_robot_interfaces__msg__PoseCommand));
  bool success = my_robot_interfaces__msg__PoseCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_robot_interfaces__msg__PoseCommand__destroy(my_robot_interfaces__msg__PoseCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_robot_interfaces__msg__PoseCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_robot_interfaces__msg__PoseCommand__Sequence__init(my_robot_interfaces__msg__PoseCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_interfaces__msg__PoseCommand * data = NULL;

  if (size) {
    data = (my_robot_interfaces__msg__PoseCommand *)allocator.zero_allocate(size, sizeof(my_robot_interfaces__msg__PoseCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_robot_interfaces__msg__PoseCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_robot_interfaces__msg__PoseCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_robot_interfaces__msg__PoseCommand__Sequence__fini(my_robot_interfaces__msg__PoseCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_robot_interfaces__msg__PoseCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_robot_interfaces__msg__PoseCommand__Sequence *
my_robot_interfaces__msg__PoseCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_interfaces__msg__PoseCommand__Sequence * array = (my_robot_interfaces__msg__PoseCommand__Sequence *)allocator.allocate(sizeof(my_robot_interfaces__msg__PoseCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_robot_interfaces__msg__PoseCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_robot_interfaces__msg__PoseCommand__Sequence__destroy(my_robot_interfaces__msg__PoseCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_robot_interfaces__msg__PoseCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_robot_interfaces__msg__PoseCommand__Sequence__are_equal(const my_robot_interfaces__msg__PoseCommand__Sequence * lhs, const my_robot_interfaces__msg__PoseCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_robot_interfaces__msg__PoseCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_robot_interfaces__msg__PoseCommand__Sequence__copy(
  const my_robot_interfaces__msg__PoseCommand__Sequence * input,
  my_robot_interfaces__msg__PoseCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_robot_interfaces__msg__PoseCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_robot_interfaces__msg__PoseCommand * data =
      (my_robot_interfaces__msg__PoseCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_robot_interfaces__msg__PoseCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_robot_interfaces__msg__PoseCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_robot_interfaces__msg__PoseCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
