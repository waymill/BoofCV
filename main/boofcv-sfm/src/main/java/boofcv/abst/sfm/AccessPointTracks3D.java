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

package boofcv.abst.sfm;

import georegression.struct.point.Point3D_F64;

/**
 * Provides information on point feature based SFM tracking algorithm
 *
 * @author Peter Abeles
 */
public interface AccessPointTracks3D extends AccessPointTracks {

	/**
	 * Returns the 3D location of the active track.
	 *
	 * @param index The track's index in the active list
	 * @param world The world coordinate of the track
	 * @return true if there's a location estimate or false if there isn't
	 */
	boolean getTrackWorld3D(int index , Point3D_F64 world );
}
