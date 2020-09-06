/*
 * Copyright (c) 2011-2020, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.alg.sfm.structure2;

import boofcv.alg.sfm.structure2.PairwiseImageGraph2.Motion;
import boofcv.alg.sfm.structure2.SceneWorkingGraph.View;
import lombok.Getter;
import org.ddogleg.sorting.QuickSort_F64;
import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_F64;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static boofcv.misc.BoofMiscOps.assertBoof;

/**
 * Selects a subset of views from a {@link SceneWorkingGraph} as the first step before performing local bundle
 * adjustment. The goal is to select the subset of views which would contribute the most to the target view's
 * state estimate. To keep computational limits in check the user needs to specifies a maximum number of views.
 *
 * Every connection between views is assigned a score. The goal is to find the subset of views which maximizes
 * the minimum score across all connections between views. This is approximated using the greedy algorithm below:
 *
 * Summary:
 * <ol>
 *     <li>Set of views will be the target's neighbors and their neighbors</li>
 *     <li>Create a list of all edges and their scores that connect these</li>
 *     <li>Repeat the loop below until only N views remain</li>
 *     <ol>
 *     <li>Select the edge with the lowest score</li>
 *     <li>Pick one of the views it is connected to be prune based on their connections</li>
 *     <li>Remove the view and all the edges connected to it</li>
 *     </ol>
 * </ol>
 *
 * Care is taken so that not all the direct neighbors are removed by setting a {@link #minNeighbors minimum number}
 * that must remain at the end. If all neighbors are removed then there is the possibility that all the remaining
 * nodes will not see the same features as the target.
 *
 * <p>
 * WARNING: There is a known flaw where a multiple disconnected graphs can be created in a poorly connected graph<br>
 * </p>
 *
 * @author Peter Abeles
 */
public class SelectNeighborsAroundView {

	/** Moving number of views in the constructed working graph. */
	public int maxViews = 10;

	/** When a view is scored for removal the N-worstOfTop connection is used for the score. */
	public int worstOfTop = 3;

	/** There should be at least this number of direct neighbors in the final list */
	public int minNeighbors = 2;

	/** Score function used to evaluate which motions should be used */
	public PairwiseGraphUtils.ScoreMotion scoreMotion = new PairwiseGraphUtils.DefaultScoreMotion();

	/** Copy of the local scene which can be independently optimized */
	protected @Getter final SceneWorkingGraph localWorking = new SceneWorkingGraph();

	//-------------- Internal Working Space

	// Keeps track of how many direct connections to the target remain
	int remainingNeighbors;

	// List of all views that it will consider for inclusion in the sub-graph
	List<View> candidates = new ArrayList<>();
	// Fast look up of candidates
	Map<String,View> lookup = new HashMap<>();
	// List of all edges with scores
	FastQueue<EdgeScore> edges = new FastQueue<>(EdgeScore::new);

	// storage for the score of connected edges
	GrowQueue_F64 connScores = new GrowQueue_F64();
	QuickSort_F64 sorter = new QuickSort_F64();

	/**
	 * Computes a local graph with the view as a seed. Local graph can be accessed by calling {@link #getLocalWorking()}
	 *
	 * @param target (Input) The view that a local graph is to be built around
	 * @param working (Input) A graph of the entire scene that a sub graph is to be made from
	 */
	public void process(View target , SceneWorkingGraph working) {
		edges.reset();
		candidates.clear();
		lookup.clear();
		localWorking.reset();

		// Create the initial list of candidate views for inclusion in the sub graph
		addNeighbors2(target, working);
		// Prune the number of views down to the minimal number
		pruneViews(target);

		// Create the local working graph that can be optimized
		candidates.add(target); // now add the target view to make it easier to see if a view is in the graph
		localWorking.addView(target.pview);
		for (int i = 0; i < candidates.size(); i++) {
			localWorking.addView(candidates.get(i).pview);
			// TODO If it contains inlier info add that but prune info on views not in this graph
		}
		// TODO look for nodes that were not included in any inlier list and create a new one based on their
		// neighbors that might not be candidates
	}

	/**
	 * Adds target's neighbors and their neighbors
	 */
	private void addNeighbors2(View target, SceneWorkingGraph working) {
		localWorking.reset();
		remainingNeighbors = target.pview.connections.size;
		for (int connIdx = 0; connIdx < target.pview.connections.size; connIdx++) {
			Motion m = target.pview.connections.get(connIdx);
			View o = working.lookupView(m.other(target.pview).id);

			candidates.add(o);
			lookup.put(o.pview.id,o);
			addEdge(m);
		}

		for (int candIdx = candidates.size()-1; candIdx >= 0; candIdx--) {
			View c = candidates.get(candIdx);
			for (int connIdx = 0; connIdx < c.pview.connections.size; connIdx++) {
				Motion m = c.pview.connections.get(connIdx);
				View o = working.lookupView(m.other(c.pview).id);
				if( lookup.containsKey(o.pview.id) )
					continue;
				candidates.add(c);
				lookup.put(o.pview.id,o);
				addEdge(m);
			}
		}
	}

	/**
	 * Reduces the candidate size until the requested maximum number of views has been meet
	 */
	private void pruneViews(View target) {
		// maxViews-1 because the target is not in the candidates list
		while( candidates.size() > maxViews-1 ) {

			// Search for the edge with the lowest score. The reason we don't just sort once and be done with it
			// is that each time we remove an element from the list that's an O(N) operation or remove swap O(1)
			// but need to re-sort it
			int lowestIndex = -1;
			double lowestScore = Double.MAX_VALUE;
			for (int i = 0; i < edges.size; i++) {
				EdgeScore s = edges.get(i);
				if( s.score >= lowestScore )
					continue;
				// See if too many neighbors have been removed and that it should keep the remaining
				if( remainingNeighbors <= minNeighbors && s.m.isConnected(target.pview))
					continue;
				// TODO Check here if removing the node would cause a split. If so don't mark it as the best
				lowestScore = s.score;
				lowestIndex = i;
			}

			if( lowestIndex < 0 )
				throw new RuntimeException("Highly likely that this is miss configured. No valid candidate has been " +
						"found for removal. Is 'minNeighbors' >= maxViews-1?");

			Motion m = edges.get(lowestIndex).m;

			if( m.isConnected(target.pview) ) {
				// Remove a neighbor of the target. No need to select which one to remove
				remainingNeighbors--;
				removeCandidateNode(m.other(target.pview).id);
			} else {
				double scoreSrc = scoreForRemoval(m.src, m);
				double scoreDst = scoreForRemoval(m.dst, m);
				removeCandidateNode(((scoreSrc < scoreDst) ? m.src : m.dst).id);
			}

			// WARNING: This should be made a bit less naive by considering if removing a View would cause other
			// Views to have no path to the target. This could probably be done efficiently by saving a reference
			// towards the target view

			// WARNING: There is nothing stopping it from pruning all of the target's neighbors also!
		}
	}

	/**
	 * Score the quality of a View based on the worst score of its top N connections.
	 * @param v The view being considered for removal
	 * @param ignore skips this motion
	 * @return score
	 */
	private double scoreForRemoval( PairwiseImageGraph2.View v, Motion ignore ) {
		connScores.reset();

		for (int connIdx = 0; connIdx < v.connections.size; connIdx++) {
			Motion m = v.connections.get(connIdx);
			String o = m.other(v).id;
			if( !lookup.containsKey(o) || m == ignore ) {
				continue;
			}
			connScores.add(scoreMotion.score(m));
		}
		if( connScores.size==0)
			return 0.0;

		connScores.sort(sorter);
		int idx = Math.max(0,connScores.size-worstOfTop);
		return connScores.get(idx);
	}

	/**
	 * Removes the specified view from the candidate list and then searches for all of its
	 * edges in the edge list and removes those
	 */
	private void removeCandidateNode(String id ) {
		View v = lookup.remove(id);
		assertBoof(candidates.remove(v));

		// Remove all edges that point to this view in the edge list
		for (int connIdx = 0; connIdx < v.pview.connections.size; connIdx++) {
			Motion m = v.pview.connections.get(connIdx);
			View o = lookup.get(m.other(v.pview).id);
			if( null == o ) {
				continue;
			}

			// If by removing the target one of it's connections is now an orphan remove that note
			// TODO see comment about graph splits. This should be handled by that logic and this removed
			if( isOrphan(o) ) {
				lookup.remove(o.pview.id);
				assertBoof(candidates.remove(o));
			}

			boolean found = false;
			for (int i = 0; i < edges.size; i++) {
				if( edges.get(i).m != m )
					continue;
				edges.removeSwap(i);
				found = true;
				break;
			}
			assertBoof(found,"No matching edge found. BUG");
		}
	}

	/**
	 * Checks to see if the View has any connections to another candidate
	 */
	private boolean isOrphan(View v) {
		for (int connIdx = 0; connIdx < v.pview.connections.size; connIdx++) {
			Motion m = v.pview.connections.get(connIdx);
			String o = m.other(v.pview).id;
			if( lookup.containsKey(o) )
				return false;
		}
		return true;
	}

	private void addEdge(Motion m) {
		EdgeScore edgeScore = edges.grow();
		edgeScore.m = m;
		edgeScore.score = scoreMotion.score(m);
	}

	private static class EdgeScore {
		// The motion this edge references
		Motion m;
		// quality of geometric information in this edge. Higher is better
		double score;
	}
}
