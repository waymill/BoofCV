/*
 * Copyright 2011 Peter Abeles
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

package gecv.alg.filter.convolve.normalized;

import gecv.misc.CodeGeneratorUtil;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

/**
 * Code generator which creates re-normalizing convolution code
 *
 * @author Peter Abeles
 */
public class GenerateConvolveNormalized_JustBorder {
	String className = "ConvolveNormalized_JustBorder";

	PrintStream out;

	public GenerateConvolveNormalized_JustBorder() throws FileNotFoundException {
		out = new PrintStream(new FileOutputStream(className + ".java"));
	}

	public void generate() {
		printPreamble();
		printAllOps("F32", "ImageFloat32","ImageFloat32","float","float","float","float","");
		printAllOps("I32", "ImageUInt8","ImageInt8","int","byte","byte","int"," & 0xFF");
		printAllOps("I32", "ImageSInt16","ImageInt16","int","short","short","int","");
		out.println("}");
	}

	private void printPreamble() {
		out.print(CodeGeneratorUtil.copyright);
		out.print("package gecv.alg.filter.convolve.normalized;\n" +
				"\n" +
				"import gecv.struct.convolve.Kernel2D_F32;\n" +
				"import gecv.struct.convolve.Kernel2D_I32;\n" +
				"import gecv.struct.convolve.Kernel1D_F32;\n" +
				"import gecv.struct.convolve.Kernel1D_I32;\n" +
				"import gecv.struct.image.*;\n"+
				"\n" +
				"/**\n" +
				" * <p>\n" +
				" * Covolves a 1D kernel in the horizontal or vertical direction across an image's border only, while re-normalizing the\n" +
				" * kernel sum to one.  The kernel MUST be smaller than the image.\n" +
				" * </p>\n" +
				" * \n" +
				" * <p>\n" +
				" * NOTE: Do not modify.  Automatically generated by {@link GenerateConvolveNormalized_JustBorder}.\n" +
				" * </p>\n" +
				" * \n" +
				" * @author Peter Abeles\n" +
				" */\n" +
				"@SuppressWarnings({\"ForLoopReplaceableByForEach\"})\n" +
				"public class "+className+" {\n\n");
	}

	private void printAllOps( String kernelType , String inputType , String outputType ,
								  String kernelData, String inputData, String outputData ,
								  String sumType, String  bitWiseOp )
	{
		printHorizontal(kernelType,inputType,outputType,kernelData,inputData,outputData,sumType,bitWiseOp);
		printVertical(kernelType,inputType,outputType,kernelData,inputData,outputData,sumType,bitWiseOp);
		printConvolve(kernelType,inputType,outputType,kernelData,inputData,outputData,sumType,bitWiseOp);
	}

	private void printHorizontal( String kernelType , String inputType , String outputType ,
								  String kernelData, String inputData, String outputData ,
								  String sumType, String  bitWiseOp) {
		
		String typeCast = outputData.compareTo(sumType) == 0 ? "" : "("+outputData+")";

		out.print("\tpublic static void horizontal(Kernel1D_"+kernelType+" kernel, "+inputType+" input, "+outputType+" output ) {\n" +
				"\t\tfinal "+inputData+"[] dataSrc = input.data;\n" +
				"\t\tfinal "+outputData+"[] dataDst = output.data;\n" +
				"\t\tfinal "+kernelData+"[] dataKer = kernel.data;\n" +
				"\n" +
				"\t\tfinal int radius = kernel.getRadius();\n" +
				"\t\tfinal int kernelWidth = kernel.getWidth();\n" +
				"\n" +
				"\t\tfinal int width = input.getWidth();\n" +
				"\t\tfinal int height = input.getHeight();\n" +
				"\n" +
				"\t\tfor (int i = 0; i < height; i++) {\n" +
				"\t\t\tint indexDest = output.startIndex + i * output.stride;\n" +
				"\t\t\tint j = input.startIndex + i * input.stride;\n" +
				"\t\t\tfinal int jStart = j;\n" +
				"\t\t\tint jEnd = j + radius;\n" +
				"\n" +
				"\t\t\tfor (; j < jEnd; j++) {\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\t\t\t\tint indexSrc = jStart;\n" +
				"\t\t\t\tfor (int k = kernelWidth - (radius + 1 + j - jStart); k < kernelWidth; k++) {\n" +
				"\t\t\t\t\t"+kernelData+" w = dataKer[k];\n" +
				"\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\ttotal += (dataSrc[indexSrc++]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDest++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\n" +
				"\t\t\tj += width - 2*radius;\n" +
				"\t\t\tindexDest += width - 2*radius;\n" +
				"\n" +
				"\t\t\tjEnd = jStart + width;\n" +
				"\t\t\tfor (; j < jEnd; j++) {\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\t\t\t\tint indexSrc = j - radius;\n" +
				"\t\t\t\tfinal int kEnd = jEnd - indexSrc;\n" +
				"\n" +
				"\t\t\t\tfor (int k = 0; k < kEnd; k++) {\n" +
				"\t\t\t\t\t"+kernelData+" w = dataKer[k];\n" +
				"\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\ttotal += (dataSrc[indexSrc++]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDest++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	private void printVertical( String kernelType , String inputType , String outputType ,
								String kernelData, String inputData, String outputData ,
								String sumType, String  bitWiseOp) {

		String typeCast = outputData.compareTo(sumType) == 0 ? "" : "("+outputData+")";

		out.print("\tpublic static void vertical(Kernel1D_"+kernelType+" kernel, "+inputType+" input, "+outputType+" output ) {\n" +
				"\t\tfinal "+inputData+"[] dataSrc = input.data;\n" +
				"\t\tfinal "+outputData+"[] dataDst = output.data;\n" +
				"\t\tfinal "+kernelData+"[] dataKer = kernel.data;\n" +
				"\n" +
				"\t\tfinal int radius = kernel.getRadius();\n" +
				"\t\tfinal int kernelWidth = kernel.getWidth();\n" +
				"\n" +
				"\t\tfinal int imgWidth = output.getWidth();\n" +
				"\t\tfinal int imgHeight = output.getHeight();\n" +
				"\n" +
				"\t\tfinal int yEnd = imgHeight - radius;\n" +
				"\n" +
				"\t\tfor (int y = 0; y < radius; y++) {\n" +
				"\t\t\tint indexDst = output.startIndex + y * output.stride;\n" +
				"\t\t\tint i = input.startIndex + y * input.stride;\n" +
				"\t\t\tfinal int iEnd = i + imgWidth;\n" +
				"\n" +
				"\t\t\tint kStart = radius - y;\n" +
				"\n" +
				"\t\t\t"+sumType+" weight = 0;\n" +
				"\t\t\tfor (int k = kStart; k < kernelWidth; k++) {\n" +
				"\t\t\t\tweight += dataKer[k];\n" +
				"\t\t\t}\n" +
				"\n" +
				"\t\t\tfor ( ; i < iEnd; i++) {\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\tint indexSrc = i - y * input.stride;\n" +
				"\t\t\t\tfor (int k = kStart; k < kernelWidth; k++, indexSrc += input.stride) {\n" +
				"\t\t\t\t\ttotal += (dataSrc[indexSrc]"+bitWiseOp+") * dataKer[k];\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / weight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\tfor (int y = yEnd; y < imgHeight; y++) {\n" +
				"\t\t\tint indexDst = output.startIndex + y * output.stride;\n" +
				"\t\t\tint i = input.startIndex + y * input.stride;\n" +
				"\t\t\tfinal int iEnd = i + imgWidth;\n" +
				"\n" +
				"\t\t\tint kEnd = imgHeight - (y - radius);\n" +
				"\n" +
				"\t\t\t"+sumType+" weight = 0;\n" +
				"\t\t\tfor (int k = 0; k < kEnd; k++) {\n" +
				"\t\t\t\tweight += dataKer[k];\n" +
				"\t\t\t}\n" +
				"\n" +
				"\t\t\tfor ( ; i < iEnd; i++) {\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\tint indexSrc = i - radius * input.stride;\n" +
				"\t\t\t\tfor (int k = 0; k < kEnd; k++, indexSrc += input.stride) {\n" +
				"\t\t\t\t\ttotal += (dataSrc[indexSrc]"+bitWiseOp+") * dataKer[k];\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / weight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printConvolve( String kernelType , String inputType , String outputType ,
							   String kernelData, String inputData, String outputData ,
							   String sumType, String  bitWiseOp) {

		String typeCast = outputData.compareTo(sumType) == 0 ? "" : "("+outputData+")";

		out.print("\tpublic static void convolve(Kernel2D_"+kernelType+" kernel, "+inputType+" input, "+outputType+" output ) {\n" +
				"\t\tfinal "+inputData+"[] dataSrc = input.data;\n" +
				"\t\tfinal "+outputData+"[] dataDst = output.data;\n" +
				"\t\tfinal "+kernelData+"[] dataKer = kernel.data;\n" +
				"\n" +
				"\t\tfinal int radius = kernel.getRadius();\n" +
				"\t\tfinal int kernelWidth = kernel.getWidth();\n" +
				"\n" +
				"\t\tfinal int width = input.getWidth();\n" +
				"\t\tfinal int height = input.getHeight();\n" +
				"\n" +
				"\t\t// convolve across the left and right borders\n" +
				"\t\tfor (int y = 0; y < height; y++) {\n" +
				"\n" +
				"\t\t\tint minI = y >= radius ? -radius : -y;\n" +
				"\t\t\tint maxI = y < height - radius ?  radius : height - y - 1;\n" +
				"\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride;\n" +
				"\n" +
				"\t\t\tfor( int x = 0; x < radius; x++ ) {\n" +
				"\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\n" +
				"\t\t\t\tfor( int i = minI; i <= maxI; i++ ) {\n" +
				"\t\t\t\t\tint indexSrc = input.startIndex + (y+i)* input.stride+x;\n" +
				"\t\t\t\t\tint indexKer = (i+radius)*kernelWidth;\n" +
				"\n" +
				"\t\t\t\t\tfor( int j = -x; j <= radius; j++ ) {\n" +
				"\t\t\t\t\t\t"+kernelData+" w = dataKer[indexKer+j+radius];\n" +
				"\t\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\t\ttotal += (dataSrc[indexSrc+j]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t\t}\n" +
				"\t\t\t\t}\n" +
				"\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\n" +
				"\t\t\tindexDst = output.startIndex + y* output.stride + width-radius;\n" +
				"\t\t\tfor( int x = width-radius; x < width; x++ ) {\n" +
				"\n" +
				"\t\t\t\tint maxJ = width-x-1;\n" +
				"\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\n" +
				"\t\t\t\tfor( int i = minI; i <= maxI; i++ ) {\n" +
				"\t\t\t\t\tint indexSrc = input.startIndex + (y+i)* input.stride+x;\n" +
				"\t\t\t\t\tint indexKer = (i+radius)*kernelWidth;\n" +
				"\n" +
				"\t\t\t\t\tfor( int j = -radius; j <= maxJ; j++ ) {\n" +
				"\t\t\t\t\t\t"+kernelData+" w = dataKer[indexKer+j+radius];\n" +
				"\t\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\t\ttotal += (dataSrc[indexSrc+j]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t\t}\n" +
				"\t\t\t\t}\n" +
				"\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\t// convolve across the top border while avoiding convolving the corners again\n" +
				"\t\tfor (int y = 0; y < radius; y++) {\n" +
				"\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride+radius;\n" +
				"\n" +
				"\t\t\tfor( int x = radius; x < width-radius; x++ ) {\n" +
				"\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\n" +
				"\t\t\t\tfor( int i = -y; i <= radius; i++ ) {\n" +
				"\t\t\t\t\tint indexSrc = input.startIndex + (y+i)* input.stride+x;\n" +
				"\t\t\t\t\tint indexKer = (i+radius)*kernelWidth;\n" +
				"\n" +
				"\t\t\t\t\tfor( int j = -radius; j <= radius; j++ ) {\n" +
				"\t\t\t\t\t\t"+kernelData+" w = dataKer[indexKer+j+radius];\n" +
				"\t\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\t\ttotal += (dataSrc[indexSrc + j]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t\t}\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\t// convolve across the bottom border\n" +
				"\t\tfor (int y = height-radius; y < height; y++) {\n" +
				"\n" +
				"\t\t\tint maxI = height - y - 1;\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride+radius;\n" +
				"\n" +
				"\t\t\tfor( int x = radius; x < width-radius; x++ ) {\n" +
				"\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\t"+sumType+" totalWeight = 0;\n" +
				"\n" +
				"\t\t\t\tfor( int i = -radius; i <= maxI; i++ ) {\n" +
				"\t\t\t\t\tint indexSrc = input.startIndex + (y+i)* input.stride+x;\n" +
				"\t\t\t\t\tint indexKer = (i+radius)*kernelWidth;\n" +
				"\n" +
				"\t\t\t\t\tfor( int j = -radius; j <= radius; j++ ) {\n" +
				"\t\t\t\t\t\t"+kernelData+" w = dataKer[indexKer+j+radius];\n" +
				"\t\t\t\t\t\ttotalWeight += w;\n" +
				"\t\t\t\t\t\ttotal += (dataSrc[indexSrc + j]"+bitWiseOp+") * w;\n" +
				"\t\t\t\t\t}\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\tdataDst[indexDst++] = "+typeCast+"(total / totalWeight);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public static void main(String args[]) throws FileNotFoundException {
		GenerateConvolveNormalized_JustBorder gen = new GenerateConvolveNormalized_JustBorder();
		gen.generate();
	}
}
