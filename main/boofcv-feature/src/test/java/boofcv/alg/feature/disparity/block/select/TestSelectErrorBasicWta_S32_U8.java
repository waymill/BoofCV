/*
 * Copyright (c) 2011-2019, Peter Abeles. All Rights Reserved.
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

package boofcv.alg.feature.disparity.block.select;

import boofcv.alg.feature.disparity.block.DisparitySelect;
import boofcv.struct.image.GrayU8;

/**
 * @author Peter Abeles
 */
public class TestSelectErrorBasicWta_S32_U8 extends CheckBasicSelectDisparity.ScoreError<int[],GrayU8> {

	TestSelectErrorBasicWta_S32_U8() {
		super(int[].class,GrayU8.class);
	}

	@Override
	public DisparitySelect<int[],GrayU8> createAlg() {
		return new SelectErrorBasicWta_S32_U8();
	}
}