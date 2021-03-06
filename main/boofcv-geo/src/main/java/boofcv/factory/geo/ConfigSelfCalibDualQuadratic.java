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

package boofcv.factory.geo;

import boofcv.alg.geo.selfcalib.SelfCalibrationLinearDualQuadratic;
import boofcv.struct.Configuration;

import static boofcv.misc.BoofMiscOps.assertBoof;

/**
 * Configuration for {@link SelfCalibrationLinearDualQuadratic}. Note that the principle point is always assumed
 * to be zero. You really should read the documentation for this class.
 *
 * @author Peter Abeles
 */
public class ConfigSelfCalibDualQuadratic implements Configuration {

	/** The skew is assumed to be zero */
	public boolean zeroSkew = true;
	/** The aspect ratio is assumed to be known, e.g. fy = ratio*fx; */
	public boolean knownAspectRatio = true;
	/** The assumed aspect ratio. Only used if {@link #knownAspectRatio} is true */
	public double aspectRatio = 1.0;

	@Override
	public void checkValidity() {
		assertBoof(aspectRatio > 0);
		assertBoof( knownAspectRatio && zeroSkew , "If aspect ratio is known then zero skew must be assumed");
	}

	public void setTo( ConfigSelfCalibDualQuadratic src ) {
		this.zeroSkew = src.zeroSkew;
		this.knownAspectRatio = src.knownAspectRatio;
		this.aspectRatio = src.aspectRatio;
	}
}
