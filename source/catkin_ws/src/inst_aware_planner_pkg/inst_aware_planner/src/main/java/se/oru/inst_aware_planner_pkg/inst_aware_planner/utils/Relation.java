package se.oru.inst_aware_planner_pkg.inst_aware_planner.utils;

import java.util.ArrayList;
import java.util.List;

public class Relation<T, S> {
	private List<Pair<T, S>> relation;

	public Relation() {
		this.relation = new ArrayList<Pair<T, S>>();
	}

	public void addRelation(T el1, S el2) {
		relation.add(new Pair<T, S>(el1, el2));
	}

	public List<Pair<T, S>> getRelation() {
		return relation;
	}

	public boolean containsPair(T obj1, S obj2) {
		return relation.contains(new Pair<T, S>(obj1, obj2));
	}

	public Integer countKeyCardinality(T key) {
		Integer keyCount = 0;
		for (Pair<T, S> pair : relation) {
			if (pair.getFirst() == key) {
				keyCount++;
			}
		}

		return keyCount;
	}

	// Get all first elements from specified second element in a relation
	public List<T> getAllFirsts(S second) {
		List<T> firsts = new ArrayList<T>();

		for (Pair<T, S> pair : relation) {
			if (pair.getSecond().equals(second)) {
				firsts.add(pair.getFirst());
			}
		}

		return firsts;
	}

	// Get all second elements from specified first element in a relation
	public List<S> getAllSeconds(T first) {
		List<S> seconds = new ArrayList<S>();

		for (Pair<T, S> pair : relation) {
			if (pair.getFirst().equals(first)) {
				seconds.add(pair.getSecond());
			}
		}

		return seconds;
	}
}
