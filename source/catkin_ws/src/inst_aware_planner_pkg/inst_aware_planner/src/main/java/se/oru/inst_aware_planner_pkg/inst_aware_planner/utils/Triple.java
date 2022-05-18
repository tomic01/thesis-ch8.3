package se.oru.inst_aware_planner_pkg.inst_aware_planner.utils;

// TODO: This should not be general triple class, 
// but more concrete institution_triple that have to 
// check its types Role, Act and Art/Role
// or a superclass of a concrete institution_triple

public class Triple<T, S, K> {
	private T first;
	private S second;
	private K third;

	public Triple(T pair1, S pair2, K pair3) {

		this.first = pair1;
		this.second = pair2;
		this.third = pair3;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;

		@SuppressWarnings("rawtypes")
		Triple other = (Triple) obj;

		// Checks the first element
		if (first == null) {
			if (other.first != null)
				return false;
		} else if (!first.equals(other.first))
			return false;

		// Checks the second element
		if (second == null) {
			if (other.second != null)
				return false;
		} else if (!second.equals(other.second))
			return false;

		// Checks the third element
		if (third == null) {
			if (other.third != null)
				return false;
		} else if (!third.equals(other.third))
			return false;

		// If all checks passes:
		return true;

	}

	public T getFirst() {
		return first;
	}

	public S getSecond() {
		return second;
	}

	public K getThird() {
		return third;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((first == null) ? 0 : first.hashCode());
		result = prime * result + ((second == null) ? 0 : second.hashCode());
		result = prime * result + ((third == null) ? 0 : third.hashCode());
		return result;
	}

	@Override
	public String toString() {
		return new String("(" + first.toString() + "," + second.toString() + "," + third.toString() + ")");
	}

}
