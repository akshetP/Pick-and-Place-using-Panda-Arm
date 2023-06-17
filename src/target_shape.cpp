/**
	MIT License

	Copyright (c) 2023 Hei Yin Wong, Agung Nuza Dwiputra, Akshet Patel

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 **/


/**
 * @file target_shape.cpp
 * @author Hei Yin Wong, Agung Nuza Dwiputra, Akshet Patel
 * @date 15.02.2023
 * @brief implementation of TargetShape
 * @defgroup cw3_class CW3 Class
 */

#include <target_shape.h>

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Constructor for the TargetShape class.
 * @param x The x-coordinate of the shape in space.
 * @param y The y-coordinate of the shape in space.
 * @param z The z-coordinate of the shape in space.
 * @param num_of_points The number of points that make up the shape.
 * @param shape_type The type of shape.
 * @param zone The zone in which the shape is located.
 */
TargetShape::TargetShape(double x, double y, double z, int num_of_points, 
std::string shape_type,int zone)
{
    x_ = x;
    y_ = y;
    z_ = z;
    picked_ = false;
    num_of_points_ = num_of_points;
    shape_type_ = shape_type;
    zone_ = zone;
}

// Getters

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for X-coordinate of target shape position
 * @return X-coordinate of target shape position
 */
double TargetShape::getX()
{
    return x_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for Y-coordinate of target shape position
 * @return Y-coordinate of target shape position
 */
double TargetShape::getY()
{
    return y_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for Z-coordinate of target shape position
 * @return Z-coordinate of target shape position
 */
double TargetShape::getZ()
{
    return z_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for pick status of the target shape
 * @return Boolean indicating if the target shape has been picked or not
 */
bool TargetShape::getPicked()
{
    return picked_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for number of points in the shape
 * @return Number of points in the shape
 */
int TargetShape::getNumOfPoints()
{
    return num_of_points_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for type of the shape
 * @return Type of the shape
 */
std::string TargetShape::getShapeType()
{
    return shape_type_;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Getter for the zone that the shape belongs to
 * @return The zone that the shape belongs to
 */
int TargetShape::getZone()
{
    return zone_;
}

// Setters

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for X-coordinate of target shape position
 * @param x X-coordinate of target shape position
 */
void TargetShape::setX(double x)
{
    x_ = x;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for Y-coordinate of target shape position
 * @param y Y-coordinate of target shape position
 */
void TargetShape::setY(double y)
{
    y_ = y;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for Z-coordinate of target shape position
 * @param z Z-coordinate of target shape position
 */
void TargetShape::setZ(double z)
{
    z_ = z;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for pick status of the target shape
 * @param picked Boolean indicating if the target shape has been picked or not
 */
void TargetShape::setPicked(bool picked)
{
    picked_ = picked;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for number of points in the shape
 * @param num_of_points Number of points in the shape
 */
void TargetShape::setNumOfPoints(int num_of_points)
{
    num_of_points_ = num_of_points;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for type of the shape
 * @param shape_type Type of the shape
 */
void TargetShape::setShapeType(std::string shape_type)
{
    shape_type_ = shape_type;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Setter for the zone that the shape belongs to
 * @param zone The zone that the shape belongs to
 */
void TargetShape::setZone(int zone)
{
    zone_ = zone;
}