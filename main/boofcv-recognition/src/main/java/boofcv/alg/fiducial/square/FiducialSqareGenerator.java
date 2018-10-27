/*
 * Copyright (c) 2011-2018, Peter Abeles. All Rights Reserved.
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

package boofcv.alg.fiducial.square;

import boofcv.abst.distort.FDistort;
import boofcv.alg.drawing.FiducialRenderEngine;
import boofcv.alg.filter.binary.ThresholdImageOps;
import boofcv.alg.misc.PixelMath;
import boofcv.struct.image.GrayU8;

/**
 * Generates images of square fiducials
 *
 * @author Peter Abeles
 */
public class FiducialSqareGenerator {
	FiducialRenderEngine renderer;

	// length of the white border surrounding the fiducial
	double whiteBorder=0;
	// length of the black border
	double blackBorder=0.25;

	public FiducialSqareGenerator(FiducialRenderEngine renderer) {
		this.renderer = renderer;
	}

	public void generate(GrayU8 image ) {
		renderer.init();

		// make sure the image is square and divisible by 8
		int s = image.width - (image.width%8);
		if( image.width != s || image.height != s ) {
			GrayU8 tmp = new GrayU8(s, s);
			new FDistort(image, tmp).scaleExt().apply();
			image = tmp;
		}

		GrayU8 binary = ThresholdImageOps.threshold(image, null, 125, false);

		PixelMath.multiply(binary,255,binary);
		double X0 = whiteBorder+blackBorder;
		double Y0 = whiteBorder+blackBorder;

		// top and bottom
		renderer.rectangle(whiteBorder,whiteBorder,1.0-whiteBorder,Y0);
		renderer.rectangle(whiteBorder,1.0-Y0,1.0-whiteBorder,1.0-whiteBorder);

		// left and right sides
		renderer.rectangle(whiteBorder,Y0,X0,1.0-Y0);
		renderer.rectangle(1.0-X0,Y0,1.0-whiteBorder,1.0-Y0);

		// Draw the image inside
		renderer.draw(image,X0,Y0,1.0-X0,1.0-Y0);
	}

	public double getWhiteBorder() {
		return whiteBorder;
	}

	public void setWhiteBorder(double whiteBorder) {
		this.whiteBorder = whiteBorder;
	}

	public double getBlackBorder() {
		return blackBorder;
	}

	public void setBlackBorder(double blackBorder) {
		this.blackBorder = blackBorder;
	}
}
