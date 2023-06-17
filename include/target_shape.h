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
 * @file target_shape.h
 * @author Hei Yin Wong, Agung Nuza Dwiputra, Akshet Patel
 * @date 21.04.2023
 * @brief header file for TargetShape, store the detected Basket or Cube
 * @defgroup cw1_class CW1 Class
 */

#ifndef TARGET_SHAPE_H_
#define TARGET_SHAPE_H_

#include <string>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
class TargetShape
{
private:
    /* data */
    double x_;
    double y_;
    double z_;
    bool picked_;
    int num_of_points_;
    std::string shape_type_;
    int zone_;
public:

		/**
		 * @brief Constructor for TargetShape class
		 * @param x x-coordinate of target shape
		 * @param y y-coordinate of target shape
		 * @param z z-coordinate of target shape
		 * @param num_of_points number of points in the target shape
		 * @param shape_type type of target shape
		 * @param zone zone number of target shape
		 */
    TargetShape(double x, double y, double z, int num_of_points, 
		std::string shape_type,int zone);

    // Getters

		/**
		 * @brief Get x-coordinate of target shape
		 * @return x-coordinate of target shape
		 */
    double getX();

		/**
		 * @brief Get y-coordinate of target shape
		 * @return y-coordinate of target shape
		 */
    double getY();

		/**
		 * @brief Get z-coordinate of target shape
		 * @return z-coordinate of target shape
		 */
    double getZ();

		/**
		 * @brief Get flag indicating if target shape has been picked up by the 
		 * robot
		 * @return true if target shape has been picked up, false otherwise
		 */
    bool getPicked();

		/**
		 * @brief Get number of points in the target shape
		 * @return number of points in the target shape
		 */
    int getNumOfPoints();

		/**
		 * @brief Get type of target shape
		 * @return type of target shape
		 */
    std::string getShapeType();

		/**
		 * @brief Get zone number of target shape
		 * @return zone number of target shape
		 */
    int getZone();
    

    // Setters

		/**
		 * @brief Set x-coordinate of target shape
		 * @param x new x-coordinate for target shape
		 */
    void setX(double x);

		/**
		 * @brief Set y-coordinate of target shape
		 * @param y new y-coordinate for target shape
		 */
    void setY(double y);

		/**
		 * @brief Set z-coordinate of target shape
		 * @param z new z-coordinate for target shape
		 */
    void setZ(double z);

		/**
		 * @brief Set flag indicating if target shape has been picked up by the 
		 * robot
		 * @param picked true if target shape has been picked up, false otherwise
		 */
    void setPicked(bool picked);

		/**
		 * @brief Set number of points in the target shape
		 * @param num_of_points new number of points for the target shape
		 */
    void setNumOfPoints(int num_of_points);

		/**
		 * @brief Set type of target shape
		 * @param shape_type new type for the target shape
     */
    void setShapeType(std::string shape_type);

		/**
		 * @brief Set the zone of the target shape
		 * @param zone An integer representing the zone where the target shape is 
		 * located.
     */
    void setZone(int zone);

};
#endif 