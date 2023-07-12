package org.firstinspires.ftc.teamcode;

import java.util.Vector;

/**
 * MovingAverage implements a moving average window filter for use with data
 * values that are a little bit noisy.  For example, if an algorithm makes a
 * decision based on an analog input voltage, that voltage will likely contain
 * a little bit of noise. A moving average window is a simple way to smooth the
 * the data value before using it.
 *
 * @author  error404
 *
 */
public class MovingAverage
{
    /**
	 * Use a Vector class to implement the window of values to average.
	 */
	private final Vector<Double> myVector = new Vector<Double>();

    /**
	 * Keep track of the size and the sum as we go along
	 */
	private int mySize;
    private double mySum;

	/**
	 * Initialize a moving average window to accept a certain number of values.
	 *
	 * @param  theSize  the length or number of values to keep in the moving average
	 *                  window.  This must be greater than 0.0
 	 */
    public MovingAverage(int theSize)
	{
        assert theSize > 0 : "Window size must be a positive integer";
        mySize = theSize;
		mySum = 0;
    }

	/**
	 * Adds a new number to the moving average object.  When the new number is
	 * added, the oldest number is removed if the desired window size has been
	 * achieved.
	 *
	 * @param  num  the number to add to the moving average
	 */
    public void add(double num)
	{
		/* udate the sum */
        mySum += num;

		/* add the value to the window */
        myVector.add(new Double(num));

        /* if the window is too big, prune the oldest value.  Since the <code>add</code>
		 * method for <code>Vector</code> adds the new value to the end, the oldest
		 * value is the first one in the list.  Thus, the oldest will be at the
		 * first position.
		 */
		if (myVector.size() > mySize)
		{
			/* simulataneously remove the value and adjust the sum accordingly */
            mySum -= myVector.remove(0).doubleValue();
        }
    }

	/**
	 * Gets the average of the values in the window.  If there are no
	 * items in the window, then 0.0 is returned for the average.
	 *
	 * @return      the average of the values in the window. Return 0.0 if the
	 *              window is empty
	 */
    public double getValue()
	{
		double theAverage = 0;

        if (myVector.size() > 0)
		{
			theAverage = mySum / myVector.size();
		}

        return theAverage;
    }
}
