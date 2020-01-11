package frc.ravenhardware;

import java.util.Comparator;
import java.util.LinkedList;

public class BufferedValue {
	protected int listSize = 29;
	
	LinkedList<Double> values;
	
	public BufferedValue() {
		values = new LinkedList<Double>();
    }
    
    public BufferedValue(int listSize) {
        values = new LinkedList<Double>();

        this.listSize = listSize;
    }
	
	// Adds the current value to the list, and
	// removes the first item if the list is larger than the list size.
	public void maintainState(double currentValue) {
		values.add(currentValue);
		
		if (values.size() > listSize) {
			values.remove();
		}
    }
    
    public void setSize(int size) {
        this.listSize = size;
    }
	
	public double getMean() {
		double cumulativeValue = 0;
		
		for (Double value : values) {
			cumulativeValue += value;
		}
				
		return cumulativeValue / values.size();
    }
    
    public double getMedian() {
		double medianValue = 0;
		Comparator<Double> comp = Comparator.naturalOrder();
		medianValue = median(values, comp);
		
		return medianValue;
	}
	
	public void traverse() {
    	for (double value : values) {
    		System.out.print(value + " ");
    	}
	}
	
		// Below here: code shamelessly copied from the internet.
	// From: https://stackoverflow.com/questions/11955728/how-to-calculate-the-median-of-an-array
	// (Answer by Bruce Feist)
	public static <T extends Number> double median(LinkedList<T> coll, Comparator<T> comp) {
		double result;
		int n = coll.size()/2;
		
		if (coll.size() % 2 == 0)  // even number of items; find the middle two and average them
		   result = (nth(coll, n-1, comp).doubleValue() + nth(coll, n, comp).doubleValue()) / 2.0;
		else                      // odd number of items; return the one in the middle
		   result = nth(coll, n, comp).doubleValue();
		   
		return result;
	 } // median(coll)
	 
	 
  
	 /*****************
	 * @param coll a collection of Comparable objects
	 * @param n  the position of the desired object, using the ordering defined on the list elements
	 * @return the nth smallest object
	 *******************/
		 
	 public static <T> T nth(LinkedList<T> coll, int n, Comparator<T> comp) {
		T result, pivot;
		LinkedList<T> underPivot = new LinkedList<>(), overPivot = new LinkedList<>(), equalPivot = new LinkedList<>();
		
		// choosing a pivot is a whole topic in itself.
		// this implementation uses the simple strategy of grabbing something from the middle of the ArrayList.
		
		pivot = coll.get(n/2);
		
		// split coll into 3 lists based on comparison with the pivot
		
		for (T obj : coll) {
		   int order = comp.compare(obj, pivot);
		   
		   if (order < 0)        // obj < pivot
			  underPivot.add(obj);
		   else if (order > 0)   // obj > pivot
			  overPivot.add(obj);
		   else                  // obj = pivot
			  equalPivot.add(obj);
		} // for each obj in coll
		
		// recurse on the appropriate list
		
		if (n < underPivot.size())
		   result = nth(underPivot, n, comp);
		else if (n < underPivot.size() + equalPivot.size()) // equal to pivot; just return it
		   result = pivot;
		else  // everything in underPivot and equalPivot is too small.  Adjust n accordingly in the recursion.
		   result = nth(overPivot, n - underPivot.size() - equalPivot.size(), comp);
		   
		return result;
	 } // nth(coll, n)

}