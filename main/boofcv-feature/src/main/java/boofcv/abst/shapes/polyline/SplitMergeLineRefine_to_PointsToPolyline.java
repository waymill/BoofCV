/*
 * Copyright (c) 2011-2017, Peter Abeles. All Rights Reserved.
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

package boofcv.abst.shapes.polyline;

import boofcv.alg.shapes.polyline.MinimizeEnergyPrune;
import boofcv.alg.shapes.polyline.RefinePolyLineCorner;
import boofcv.alg.shapes.polyline.splitmerge.SplitMergeLineFit;
import boofcv.alg.shapes.polyline.splitmerge.SplitMergeLineFitLoop;
import boofcv.alg.shapes.polyline.splitmerge.SplitMergeLineFitSegment;
import boofcv.struct.ConfigLength;
import georegression.struct.point.Point2D_I32;
import org.ddogleg.struct.GrowQueue_I32;

import java.util.List;

/**
 * Wrapper around {@link SplitMergeLineFit} and other refinement algorithms for {@link PointsToPolyline}.
 *
 * @author Peter Abeles
 */
public class SplitMergeLineRefine_to_PointsToPolyline implements PointsToPolyline {

	// reject the number of sides found is greater than this amount
	int maxVertexes = Integer.MAX_VALUE;

	// standard split merge algorithm
	SplitMergeLineFit splitMerge;
	// refine corner location
	RefinePolyLineCorner refine;
	// removes extra corners
	private GrowQueue_I32 pruned = new GrowQueue_I32(); // corners after pruning
	private MinimizeEnergyPrune pruner;

	public SplitMergeLineRefine_to_PointsToPolyline(double splitFraction,
													ConfigLength minimumSplit,
													int maxIterations,
													int refineIterations,
													double pruneSplitPentially,
													boolean loop )
	{
		if( loop ) {
			splitMerge = new SplitMergeLineFitLoop(splitFraction, minimumSplit, maxIterations);
		} else {
			splitMerge = new SplitMergeLineFitSegment(splitFraction, minimumSplit, maxIterations);
		}

		if( refineIterations > 0 ) {
			refine = new RefinePolyLineCorner(loop,refineIterations);
		}

		if( pruneSplitPentially > 0 ) {
			pruner = new MinimizeEnergyPrune(pruneSplitPentially);
		}
	}


	@Override
	public boolean process(List<Point2D_I32> input, GrowQueue_I32 vertexes) {
		if( !splitMerge.process(input,vertexes) ) {
			return false;
		}

		if( refine != null && !refine.fit(input,vertexes)) {
			return false;
		}

		if( pruner != null && pruner.prune(input,vertexes,pruned) ) {
			vertexes.setTo(pruned);
		}

		return vertexes.size <= maxVertexes;
	}

	@Override
	public void setMaxVertexes(int maximum) {
		this.maxVertexes = maximum;
		// detect more than the max. Prune will reduce the number of corners later on
		splitMerge.setAbortSplits(maximum*2);
	}

	@Override
	public int getMaxVertexes() {
		return maxVertexes;
	}

	@Override
	public boolean isLoop() {
		return splitMerge instanceof SplitMergeLineFitLoop;
	}
}