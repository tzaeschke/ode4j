/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.drawstuff.internal;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PushbackInputStream;
import java.lang.Math;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

import static org.ode4j.drawstuff.DrawStuff.*;

/**
 *
 * simple graphics.
 * 
 * the following command line flags can be used (typically under unix)
 * 	-notex		Do not use any textures
 * 	-noshadow[s]	Do not draw any shadows
 * 	-pause		Start the simulation paused
 *  -texturepath [path] Inform an alternative textures path
 *
 * TODO
 * ----
 * 
 * manage openGL state changes better
 * 
 */
//public class DrawStuff extends Swing implements All {
public class DrawStuffGL extends LwJGL implements DrawStuffApi {

	
	// ***************************************************************************
	// misc

	private static final String DEFAULT_PATH_TO_TEXTURES;
	static {
		String s = System.getProperty("file.separator");
		DEFAULT_PATH_TO_TEXTURES = ".." + s + "textures" + s;
	}

	private static final double M_PI = Math.PI;

	// constants to convert degrees to radians and the reverse
	//private static final double RAD_TO_DEG = 180.0/Math.PI;
	private static final double DEG_TO_RAD = Math.PI/180.0; 
	//#define RAD_TO_DEG (180.0/M_PI)
	//#define DEG_TO_RAD (M_PI/180.0)

	// light vector. LIGHTZ is implicitly 1
	private static final float LIGHTX = 1.0f;
	private static final float LIGHTY = 0.4f;
	//#define LIGHTX (1.0f)
	//#define LIGHTY (0.4f)

	// ground and sky
	private static final float SHADOW_INTENSITY = 0.65f;
	// ground color for when there's no texture
	private static final float GROUND_R = 0.5f;
	private static final float GROUND_G = 0.5f;
	private static final float GROUND_B = 0.3f;
	//#define SHADOW_INTENSITY (0.65f)
	//#define GROUND_R (0.5f) 	// ground color for when there's no texture
	//#define GROUND_G (0.5f)
	//#define GROUND_B (0.3f)

	private static final float ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
	private static final float ground_ofsx = 0.5f;		// offset of ground texture
	private static final float ground_ofsy = 0.5f;
	private static final float sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
	private static final float sky_height = 1.0f;		// sky height above viewpoint

	// ***************************************************************************
	// misc mathematics stuff


	private void normalizeVector3 (float[] v)//[3])
	{
		float len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
		if (len <= 0.0f) {
			v[0] = 1;
			v[1] = 0;
			v[2] = 0;
		}
		else {
			len = 1.0f / (float)Math.sqrt(len);
			v[0] *= len;
			v[1] *= len;
			v[2] *= len;
		}
	}
	
	private static void crossProduct3(float[] res, float[] a, float[] b)
	{
	  float res_0 = a[1]*b[2] - a[2]*b[1];
	  float res_1 = a[2]*b[0] - a[0]*b[2];
	  float res_2 = a[0]*b[1] - a[1]*b[0];
	  // Only assign after all the calculations are over to avoid incurring memory aliasing
	  res[0] = res_0;
	  res[1] = res_1;
	  res[2] = res_2;
	}

	//***************************************************************************
	// PPM image object

	private interface Image {

		int height();

		int width();

		ByteBuffer data();
		
	}
	
	private static class ImageIO implements Image {
		private int image_width, image_height;
		private ByteBuffer image_data;
		//public:
		//Image (String filename);
		// load from PPM file
		//  ~Image();
		@Override
		public int width() { return image_width; }
		@Override
		public int height() { return image_height; }
		@Override
		public ByteBuffer data() { return image_data; }



		// skip over whitespace and comments in a stream.

		private void skipWhiteSpace (String filename, PushbackInputStream f) throws IOException
		{
			int c,d;
			while(true) {//for(;;) {
				c = f.read();//fgetc(f);
				if (c==-1) dsError ("unexpected end of file in \"%s\"",filename);

				// skip comments
				if (c == '#') {
					do {
						d = f.read();//fgetc(f);
						if (d==-1) dsError ("unexpected end of file in \"%s\"",filename);
					} while (d != '\n');
					continue;
				}

				if (c > ' ') {
					f.unread(c);//ungetc (c,f);
					return;
				}
			}
		}


		// read a number from a stream, this return 0 if there is none (that's okay
		// because 0 is a bad value for all PPM numbers anyway).

		private int readNumber (String filename, PushbackInputStream f) throws IOException
		{
			int c,n=0;
			for(;;) {
				c = f.read();//fgetc(f);
				if (c==-1) dsError ("unexpected end of file in \"%s\"",filename);
				if (c >= '0' && c <= '9') n = n*10 + (c - '0');
				else {
					f.unread(c);//ungetc (c,f);
					return n;
				}
			}
		}


		ImageIO (String filename)
		{
			PushbackInputStream f = null;
			try {
				InputStream is = getClass().getResourceAsStream(filename); 
				if (is == null) throw new IllegalArgumentException("File not found: " + filename);
				f = new PushbackInputStream( new BufferedInputStream( is )); 
				//if (f == null) dsError ("Can't open image file `%s'",filename);

				// read in header
				//if (fgetcC(f) != 'P' || fgetcC(f) != '6')
				if (f.read() != 'P' || f.read() != '6')
					dsError ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
				skipWhiteSpace (filename,f);

				// read in image parameters
				image_width = readNumber (filename,f);
				skipWhiteSpace (filename,f);
				image_height = readNumber (filename,f);
				skipWhiteSpace (filename,f);
				int max_value = readNumber (filename,f);

				// check values
				if (image_width < 1 || image_height < 1)
					dsError ("bad image file \"%s\"",filename);
				if (max_value != 255)
					dsError ("image file \"%s\" must have color range of 255",filename);

				// read either nothing, LF (10), or CR,LF (13,10)
				int c = f.read();//fgetc(f);
				if (c == 10) {
					// LF
				}
				else if (c == 13) {
					// CR
					c = f.read();//fgetc(f);
					if (c != 10) f.unread(c);//ungetc (c,f);
				}
				else f.unread(c);//ungetc (c,f);

				// read in rest of data
				//image_data = new byte [image_width*image_height*3];
				image_data = BufferUtils.createByteBuffer(image_width*image_height*3);
				//TODO TZ read directly into direct buffer!
				byte[] buf = new byte [image_width*image_height*3];
				if (f.read (buf, 0, image_width*image_height*3 * 1) 
						!= image_width*image_height*3)
					dsError ("Can not read data from image file `%s'",filename);
				image_data.put(buf);
				image_data.flip();
			} catch (IOException e) {
				System.err.println("Error reading file: \"" + filename + "\"");
				e.printStackTrace();
				if (f != null) {
					try {
						f.close ();
					} catch (IOException e2) {
						e2.printStackTrace();
					}
				}
				throw new RuntimeException(e);
			}
		}
	}
	
	//***************************************************************************
	// Texture object.

	private static class Texture {
		private Image image;
		private int name; // GLuint TZ
		//public:
		//Texture (String filename);
		//  ~Texture();
		//void bind (int modulate);


		public Texture (String filename)
		{
			image = new ImageIO (filename);
//			GL11.glGenTextures (1,name);
			IntBuffer ib = BufferUtils.createIntBuffer(1);
			ib.put(name).flip();  //Necessary ? TZ
			GL11.glGenTextures (ib);
			name = ib.get(0);
			GL11.glBindTexture (GL11.GL_TEXTURE_2D,name);

			// set pixel unpacking mode
			GL11.glPixelStorei (GL11.GL_UNPACK_SWAP_BYTES, 0);
			GL11.glPixelStorei (GL11.GL_UNPACK_ROW_LENGTH, 0);
			GL11.glPixelStorei (GL11.GL_UNPACK_ALIGNMENT, 1);
			GL11.glPixelStorei (GL11.GL_UNPACK_SKIP_ROWS, 0);
			GL11.glPixelStorei (GL11.GL_UNPACK_SKIP_PIXELS, 0);

			//GL11.glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
			//GL11.glTexSubImage2D(GL11.GL_TEXTURE_2D, 0​, 0, 0, image.width(), image.height(), GL11.GL_BGRA, GL11.GL_UNSIGNED_BYTE, image.data());

//			glTexImage2D (GL11.GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0,
//					   GL_RGB, GL_UNSIGNED_BYTE, image->data());
			// TODO CHECK-TZ
			// GL11.GL_RGBA8
			GL11.glTexImage2D (GL11.GL_TEXTURE_2D, 0, 3, image.width(), image.height(), 0,
					GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, image.data());//ByteBuffer.wrap(image.data().array()));
			//TZ this is the original:
//			GLX.gluBuild2DMipmaps (GL11.GL_TEXTURE_2D, 3, image.width(), image.height(),
//			GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, image.data());
//			GLU.gluBuild2DMipmaps (GL11.GL_TEXTURE_2D, 3, image.width(), image.height(),
//			GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, image.data());
			// TZ Please comment this out if it doesn´t work for you.
			//  The demos will still work, albeit without textures.
			GL30.glGenerateMipmap(GL30.GL_TEXTURE_2D);
			
			// set texture parameters - will these also be bound to the texture???
			GL11.glTexParameterf (GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_S, GL11.GL_REPEAT);
			GL11.glTexParameterf (GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_T, GL11.GL_REPEAT);

			GL11.glTexParameterf (GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
			GL11.glTexParameterf (GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER,
					GL11.GL_LINEAR_MIPMAP_LINEAR);

			GL11.glTexEnvf (GL11.GL_TEXTURE_ENV, GL11.GL_TEXTURE_ENV_MODE, GL11.GL_DECAL);
		}


		//Texture::~Texture()
		//{
		//  delete image;
		//  glDeleteTextures (1,&name);
		//}
		//		@Override
		//		protected void finalize() throws Throwable {
		//			image = null;
		////			GL11.glDeleteTextures (1, name));
		//			GL11.glDeleteTextures (IntBuffer.wrap(new int[]{name}));
		//			super.finalize();
		//		}

		public void bind (boolean modulate)
		{
			GL11.glBindTexture (GL11.GL_TEXTURE_2D,name);
			GL11.glTexEnvi (GL11.GL_TEXTURE_ENV, GL11.GL_TEXTURE_ENV_MODE,
					modulate ? GL11.GL_MODULATE : GL11.GL_DECAL);
		}
	}


	//***************************************************************************
	// the current drawing state (for when the user's step function is drawing)

	private static float[] color = {0,0,0,0};	// current r,g,b,alpha color
	private static DS_TEXTURE_NUMBER tnum = DS_TEXTURE_NUMBER.DS_NONE;			// current texture number

	//***************************************************************************
	// OpenGL utility stuff

	private void setCamera (float x, float y, float z, float h, float p, float r)
	{
		GL11.glMatrixMode (GL11.GL_MODELVIEW);
		GL11.glLoadIdentity();
		GL11.glRotatef (90, 0,0,1);
		GL11.glRotatef (90, 0,1,0);
		GL11.glRotatef (r, 1,0,0);
		GL11.glRotatef (p, 0,1,0);
		GL11.glRotatef (-h, 0,0,1);
		GL11.glTranslatef (-x,-y,-z);
	}


	// sets the material color, not the light color

	private FloatBuffer light_ambient2 = BufferUtils.createFloatBuffer(4);
	private FloatBuffer light_diffuse2 = BufferUtils.createFloatBuffer(4);
	private FloatBuffer light_specular2 = BufferUtils.createFloatBuffer(4);
	private void setColor (float r, float g, float b, float alpha)
	{
		//GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
		light_ambient2.put( new float[]{ r*0.3f, g*0.3f, b*0.3f, alpha }).flip();
		light_diffuse2.put( new float[]{ r*0.7f, g*0.7f, b*0.7f, alpha }).flip();
		light_specular2.put( new float[]{ r*0.2f, g*0.2f, b*0.2f, alpha }).flip();
		GL11.glMaterialfv (GL11.GL_FRONT_AND_BACK, GL11.GL_AMBIENT, light_ambient2);
		GL11.glMaterialfv (GL11.GL_FRONT_AND_BACK, GL11.GL_DIFFUSE, light_diffuse2);
		GL11.glMaterialfv (GL11.GL_FRONT_AND_BACK, GL11.GL_SPECULAR, light_specular2);
		GL11.glMaterialf (GL11.GL_FRONT_AND_BACK, GL11.GL_SHININESS, 5.0f);
	}


	private FloatBuffer matrixF = BufferUtils.createFloatBuffer(16);
	
//	static void setTransform (final float pos[3], final float R[12])
	private void setTransform (final float[] pos, final float[] R)
	{
		//GLfloat
		float[] matrix=new float[16];
		matrix[0]=R[0];
		matrix[1]=R[4];
		matrix[2]=R[8];
		matrix[3]=0;
		matrix[4]=R[1];
		matrix[5]=R[5];
		matrix[6]=R[9];
		matrix[7]=0;
		matrix[8]=R[2];
		matrix[9]=R[6];
		matrix[10]=R[10];
		matrix[11]=0;
		matrix[12]=pos[0];
		matrix[13]=pos[1];
		matrix[14]=pos[2];
		matrix[15]=1;
		matrixF.put(matrix);
		matrixF.flip();
		GL11.glPushMatrix();
		GL11.glMultMatrixf (matrixF);
	}
	
	private DoubleBuffer matrixD = BufferUtils.createDoubleBuffer(16);
	
//	static void setTransformD (final double pos[3], final double R[12])
	private void setTransform (final DVector3C pos, final DMatrix3C R)
	{
		//GLdouble
		double[] matrix=new double[16];
		matrix[0]=R.get00();
		matrix[1]=R.get10();
		matrix[2]=R.get20();
		matrix[3]=0;
		matrix[4]=R.get01();
		matrix[5]=R.get11();
		matrix[6]=R.get21();
		matrix[7]=0;
		matrix[8]=R.get02();
		matrix[9]=R.get12();
		matrix[10]=R.get22();
		matrix[11]=0;
		matrix[12]=pos.get0();
		matrix[13]=pos.get1();
		matrix[14]=pos.get2();
		matrix[15]=1;
		matrixD.put(matrix);
		matrixD.flip();
		GL11.glPushMatrix();
		GL11.glMultMatrixd (matrixD);
	}


	// set shadow projection transform

	private FloatBuffer matrixSST = BufferUtils.createFloatBuffer(16);
	private void setShadowTransform()
	{
		//GLfloat
		float[] matrix=new float[16];
		for (int i=0; i<16; i++) matrix[i] = 0;
		matrix[0]=1;
		matrix[5]=1;
		matrix[8]=-LIGHTX;
		matrix[9]=-LIGHTY;
		matrix[15]=1;
		matrixSST.put( matrix );
//		for (int i=0; i < 16; i++) matrixSST.put(i, 0);
//		matrixSST.put(0, 1);
//		matrixSST.put(5, 1);
//		matrixSST.put(8, -LIGHTX);
//		matrixSST.put(9, -LIGHTY);
//		matrixSST.put(15, 1);
		matrixSST.flip();
		GL11.glPushMatrix();
		GL11.glMultMatrixf (matrixSST);
	}

//	static void drawConvex (float *_planes,unsigned int _planecount,
//			float *_points,
//			unsigned int _pointcount,
//			unsigned int *_polygons)
	private void drawConvex (float[] _planes, int _planecount,
			float[] _points,
			int _pointcount,
			int[] _polygons)
	{
		//unsigned 
		int polyindex=0;
		for(int i=0;i<_planecount;++i)
		{
			//unsigned 
			int pointcount=_polygons[polyindex];
			polyindex++;
			GL11.glBegin (GL11.GL_POLYGON);      
			GL11.glNormal3f(_planes[(i*4)+0],
					_planes[(i*4)+1],
					_planes[(i*4)+2]);
			for(int j=0;j<pointcount;++j)
			{
				GL11.glVertex3f(_points[_polygons[polyindex]*3],
						_points[(_polygons[polyindex]*3)+1],
						_points[(_polygons[polyindex]*3)+2]);
				polyindex++;
			}
			GL11.glEnd();
		}
	}

//	static void drawConvexD (double *_planes,unsigned int _planecount,
//			double *_points,
//			unsigned int _pointcount,
//			unsigned int *_polygons)
	private void drawConvexD (double[] _planes, int _planecount,
			double[] _points,
			int _pointcount,
			int[] _polygons)
	{
		//unsigned 
		int polyindex=0;
		for(int i=0;i<_planecount;++i)
		{
			//unsigned 
			int pointcount=_polygons[polyindex];
			polyindex++;
			GL11.glBegin (GL11.GL_POLYGON);
			GL11.glNormal3d(_planes[(i*4)+0],
					_planes[(i*4)+1],
					_planes[(i*4)+2]);
			for(int j=0;j<pointcount;++j)
			{
				GL11.glVertex3d(_points[_polygons[polyindex]*3],
						_points[(_polygons[polyindex]*3)+1],
						_points[(_polygons[polyindex]*3)+2]);
				polyindex++;
			}
			GL11.glEnd();
		}
	}

	//static void drawBox (final float sides[3])
	private void drawBox (final float[] sides)
	{
		float lx = sides[0]*0.5f;
		float ly = sides[1]*0.5f;
		float lz = sides[2]*0.5f;

		// sides
		GL11.glBegin (GL11.GL_TRIANGLE_STRIP);
		GL11.glNormal3f (-1,0,0);
		GL11.glVertex3f (-lx,-ly,-lz);
		GL11.glVertex3f (-lx,-ly,lz);
		GL11.glVertex3f (-lx,ly,-lz);
		GL11.glVertex3f (-lx,ly,lz);
		GL11.glNormal3f (0,1,0);
		GL11.glVertex3f (lx,ly,-lz);
		GL11.glVertex3f (lx,ly,lz);
		GL11.glNormal3f (1,0,0);
		GL11.glVertex3f (lx,-ly,-lz);
		GL11.glVertex3f (lx,-ly,lz);
		GL11.glNormal3f (0,-1,0);
		GL11.glVertex3f (-lx,-ly,-lz);
		GL11.glVertex3f (-lx,-ly,lz);
		GL11.glEnd();

		// top face
		GL11.glBegin (GL11.GL_TRIANGLE_FAN);
		GL11.glNormal3f (0,0,1);
		GL11.glVertex3f (-lx,-ly,lz);
		GL11.glVertex3f (lx,-ly,lz);
		GL11.glVertex3f (lx,ly,lz);
		GL11.glVertex3f (-lx,ly,lz);
		GL11.glEnd();

		// bottom face
		GL11.glBegin (GL11.GL_TRIANGLE_FAN);
		GL11.glNormal3f (0,0,-1);
		GL11.glVertex3f (-lx,-ly,-lz);
		GL11.glVertex3f (-lx,ly,-lz);
		GL11.glVertex3f (lx,ly,-lz);
		GL11.glVertex3f (lx,-ly,-lz);
		GL11.glEnd();
	}


	// This is recursively subdivides a triangular area (vertices p1,p2,p3) into
	// smaller triangles, and then draws the triangles. All triangle vertices are
	// normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
	// to be already normalized). Note this is not super-fast because it draws
	// triangles rather than triangle strips.

//	static void drawPatch (float p1[3], float p2[3], float p3[3], int level)
	private void drawPatch (float[] p1, float[] p2, float[] p3, int level)
	{
		int i;
		if (level > 0) {
			float[] q1=new float[3],q2=new float[3],q3=new float[3];		 // sub-vertices
			for (i=0; i<3; i++) {
				q1[i] = 0.5f*(p1[i]+p2[i]);
				q2[i] = 0.5f*(p2[i]+p3[i]);
				q3[i] = 0.5f*(p3[i]+p1[i]);
			}
			float length1 = (float)(1.0/Math.sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
			float length2 = (float)(1.0/Math.sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
			float length3 = (float)(1.0/Math.sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
			for (i=0; i<3; i++) {
				q1[i] *= length1;
				q2[i] *= length2;
				q3[i] *= length3;
			}
			drawPatch (p1,q1,q3,level-1);
			drawPatch (q1,p2,q2,level-1);
			drawPatch (q1,q2,q3,level-1);
			drawPatch (q3,q2,p3,level-1);
		}
		else {
			GL11.glNormal3f (p1[0],p1[1],p1[2]);
			GL11.glVertex3f (p1[0],p1[1],p1[2]);
			GL11.glNormal3f (p2[0],p2[1],p2[2]);
			GL11.glVertex3f (p2[0],p2[1],p2[2]);
			GL11.glNormal3f (p3[0],p3[1],p3[2]);
			GL11.glVertex3f (p3[0],p3[1],p3[2]);
		}
	}


	// draw a sphere of radius 1

	private int sphere_quality = 1;

	private static final float ICX = 0.525731112119133606f;
	private static final float ICZ = 0.850650808352039932f;
	private static final float[][] idata = { //GLfloat [12][3] = {
		{-ICX, 0, ICZ},
		{ICX, 0, ICZ},
		{-ICX, 0, -ICZ},
		{ICX, 0, -ICZ},
		{0, ICZ, ICX},
		{0, ICZ, -ICX},
		{0, -ICZ, ICX},
		{0, -ICZ, -ICX},
		{ICZ, ICX, 0},
		{-ICZ, ICX, 0},
		{ICZ, -ICX, 0},
		{-ICZ, -ICX, 0}
	};
	private static final int[][] index = {//[20][3] = {
		{0, 4, 1},	  {0, 9, 4},
		{9, 5, 4},	  {4, 5, 8},
		{4, 8, 1},	  {8, 10, 1},
		{8, 3, 10},   {5, 3, 8},
		{5, 2, 3},	  {2, 7, 3},
		{7, 10, 3},   {7, 6, 10},
		{7, 11, 6},   {11, 0, 6},
		{0, 1, 6},	  {6, 1, 10},
		{9, 0, 11},   {9, 11, 2},
		{9, 2, 5},	  {7, 2, 11},
	};
	private static int listnum = 0; //GLunint TZ
	private void drawSphere()
	{
		// icosahedron data for an icosahedron of radius 1.0
//		# define ICX 0.525731112119133606f
//		# define ICZ 0.850650808352039932f
		if (listnum==0) {
			listnum = GL11.glGenLists (1);
			GL11.glNewList (listnum,GL11.GL_COMPILE);
			GL11.glBegin (GL11.GL_TRIANGLES);
			for (int i=0; i<20; i++) {
//				drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
//						&idata[index[i][0]][0],sphere_quality);
				drawPatch (idata[index[i][2]],idata[index[i][1]],
						idata[index[i][0]],sphere_quality);
			}
			GL11.glEnd();
			GL11.glEndList();
		}
		GL11.glCallList (listnum);
	}


//	private static int init=0;
	private static boolean init=false;
	private static float len2,len1,scale;
	private void drawSphereShadow (float px, float py, float pz, float radius)
	{
		// calculate shadow constants based on light vector
		if (!init) {
			len2 = LIGHTX*LIGHTX + LIGHTY*LIGHTY;
			len1 = 1.0f/(float)Math.sqrt(len2);
			scale = (float) Math.sqrt(len2 + 1);
			init = true;
		}

		// map sphere center to ground plane based on light vector
		px -= LIGHTX*pz;
		py -= LIGHTY*pz;

		final float kx = 0.96592582628907f;
		final float ky = 0.25881904510252f;
		float x=radius, y=0;

		GL11.glBegin (GL11.GL_TRIANGLE_FAN);
		for (int i=0; i<24; i++) {
			// for all points on circle, scale to elongated rotated shadow and draw
			float x2 = (LIGHTX*x*scale - LIGHTY*y)*len1 + px;
			float y2 = (LIGHTY*x*scale + LIGHTX*y)*len1 + py;
			GL11.glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
			GL11.glVertex3f (x2,y2,0);

			// rotate [x,y] vector
			float xtmp = kx*x - ky*y;
			y = ky*x + kx*y;
			x = xtmp;
		}
		GL11.glEnd();
	}


//	static void drawTriangle (final float *v0, final float *v1, final float *v2, int solid)
	private void drawTriangle (final float []vAll, final int v0, final int v1, 
			final int v2, boolean solid)
	{
		float[] u=new float[3],v=new float[3],normal=new float[3];
		u[0] = vAll[v1] - vAll[v0];//v1[0] - v0[0];
		u[1] = vAll[v1+1] - vAll[v0+1];//v1[1] - v0[1];
		u[2] = vAll[v1+2] - vAll[v0+2];//v1[2] - v0[2];
		v[0] = vAll[v2] - vAll[v0];//v2[0] - v0[0];
		v[1] = vAll[v2+1] - vAll[v0+1];//v2[1] - v0[1];
		v[2] = vAll[v2+2] - vAll[v0+2];//v2[2] - v0[2];
		crossProduct3(normal,u,v);
		normalizeVector3 (normal);

		GL11.glBegin(solid ? GL11.GL_TRIANGLES : GL11.GL_LINE_STRIP);
		GL11.glNormal3f (normal[0], normal[1], normal[2]);
		GL11.glVertex3f (vAll[v0], vAll[v0+1], vAll[v0+2]);//, v0[0], v0[1], v0[2]);
		GL11.glVertex3f (vAll[v1], vAll[v1+1], vAll[v1+2]);//v1[0], v1[1], v1[2]);
		GL11.glVertex3f (vAll[v2], vAll[v2+1], vAll[v2+2]);//v2[0], v2[1], v2[2]);
		GL11.glEnd();
	}

	private void drawTriangle (final float []v0, final float []v1, 
			final float []v2, boolean solid)
	{
		float[] u=new float[3],v=new float[3],normal=new float[3];
		u[0] = v1[0] - v0[0];
		u[1] = v1[1] - v0[1];
		u[2] = v1[2] - v0[2];
		v[0] = v2[0] - v0[0];
		v[1] = v2[1] - v0[1];
		v[2] = v2[2] - v0[2];
		crossProduct3 (normal,u,v);
		normalizeVector3 (normal);

		GL11.glBegin(solid ? GL11.GL_TRIANGLES : GL11.GL_LINE_STRIP);
		GL11.glNormal3f (normal[0], normal[1], normal[2]);
		GL11.glVertex3f (v0[0], v0[1], v0[2]);
		GL11.glVertex3f (v1[0], v1[1], v1[2]);
		GL11.glVertex3f (v2[0], v2[1], v2[2]);
		GL11.glEnd();
	}

//	private static void drawTriangleD (final double []v0, final double []v1, 
//			final double []v2, boolean solid)
//	{
//		float[] u=new float[3],v=new float[3],normal=new float[3];
//		u[0] = (float) ( v1[0] - v0[0] );
//		u[1] = (float) ( v1[1] - v0[1] );
//		u[2] = (float) ( v1[2] - v0[2] );
//		v[0] = (float) ( v2[0] - v0[0] );
//		v[1] = (float) ( v2[1] - v0[1] );
//		v[2] = (float) ( v2[2] - v0[2] );
//		OdeMath.dCROSS (normal,OP.EQ,u,v);
//		normalizeVector3 (normal);
//
//		GL11.glBegin(solid ? GL11.GL_TRIANGLES : GL11.GL_LINE_STRIP);
//		GL11.glNormal3f (normal[0], normal[1], normal[2]);
//		GL11.glVertex3d (v0[0], v0[1], v0[2]);
//		GL11.glVertex3d (v1[0], v1[1], v1[2]);
//		GL11.glVertex3d (v2[0], v2[1], v2[2]);
//		GL11.glEnd();
//	}

	private void drawTriangle (final DVector3C v0, final DVector3C v1, 
			final DVector3C v2, boolean solid)
	{
		float[] u=new float[3],v=new float[3],normal=new float[3];
		u[0] = (float) ( v1.get0() - v0.get0() );
		u[1] = (float) ( v1.get1() - v0.get1() );
		u[2] = (float) ( v1.get2() - v0.get2() );
		v[0] = (float) ( v2.get0() - v0.get0() );
		v[1] = (float) ( v2.get1() - v0.get1() );
		v[2] = (float) ( v2.get2() - v0.get2() );
		crossProduct3 (normal,u,v);
		normalizeVector3 (normal);

		GL11.glBegin(solid ? GL11.GL_TRIANGLES : GL11.GL_LINE_STRIP);
		GL11.glNormal3f (normal[0], normal[1], normal[2]);
		GL11.glVertex3d (v0.get0(), v0.get1(), v0.get2());
		GL11.glVertex3d (v1.get0(), v1.get1(), v1.get2());
		GL11.glVertex3d (v2.get0(), v2.get1(), v2.get2());
		GL11.glEnd();
	}


	// draw a capped cylinder of length l and radius r, aligned along the x axis

	private int capped_cylinder_quality = 3;

	private void drawCapsule (float l, float r)
	{
		int i,j;
		float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
		// number of sides to the cylinder (divisible by 4):
		final int n = capped_cylinder_quality*4;

		l *= 0.5;
		a = (float) ((M_PI*2.0)/n);
		sa = (float) Math.sin(a);
		ca = (float) Math.cos(a);

		// draw cylinder body
		ny=1; nz=0;		  // normal vector = (0,ny,nz)
		GL11.glBegin (GL11.GL_TRIANGLE_STRIP);
		for (i=0; i<=n; i++) {
			GL11.glNormal3d (ny,nz,0);
			GL11.glVertex3d (ny*r,nz*r,l);
			GL11.glNormal3d (ny,nz,0);
			GL11.glVertex3d (ny*r,nz*r,-l);
			// rotate ny,nz
			tmp = ca*ny - sa*nz;
			nz = sa*ny + ca*nz;
			ny = tmp;
		}
		GL11.glEnd();

		// draw first cylinder cap
		start_nx = 0;
		start_ny = 1;
		for (j=0; j<(n/4); j++) {
			// get start_n2 = rotated start_n
			float start_nx2 =  ca*start_nx + sa*start_ny;
			float start_ny2 = -sa*start_nx + ca*start_ny;
			// get n=start_n and n2=start_n2
			nx = start_nx; ny = start_ny; nz = 0;
			float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
			GL11.glBegin (GL11.GL_TRIANGLE_STRIP);
			for (i=0; i<=n; i++) {
				GL11.glNormal3d (ny2,nz2,nx2);
				GL11.glVertex3d (ny2*r,nz2*r,l+nx2*r);
				GL11.glNormal3d (ny,nz,nx);
				GL11.glVertex3d (ny*r,nz*r,l+nx*r);
				// rotate n,n2
				tmp = ca*ny - sa*nz;
				nz = sa*ny + ca*nz;
				ny = tmp;
				tmp = ca*ny2- sa*nz2;
				nz2 = sa*ny2 + ca*nz2;
				ny2 = tmp;
			}
			GL11.glEnd();
			start_nx = start_nx2;
			start_ny = start_ny2;
		}

		// draw second cylinder cap
		start_nx = 0;
		start_ny = 1;
		for (j=0; j<(n/4); j++) {
			// get start_n2 = rotated start_n
			float start_nx2 = ca*start_nx - sa*start_ny;
			float start_ny2 = sa*start_nx + ca*start_ny;
			// get n=start_n and n2=start_n2
			nx = start_nx; ny = start_ny; nz = 0;
			float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
			GL11.glBegin (GL11.GL_TRIANGLE_STRIP);
			for (i=0; i<=n; i++) {
				GL11.glNormal3d (ny,nz,nx);
				GL11.glVertex3d (ny*r,nz*r,-l+nx*r);
				GL11.glNormal3d (ny2,nz2,nx2);
				GL11.glVertex3d (ny2*r,nz2*r,-l+nx2*r);
				// rotate n,n2
				tmp = ca*ny - sa*nz;
				nz = sa*ny + ca*nz;
				ny = tmp;
				tmp = ca*ny2- sa*nz2;
				nz2 = sa*ny2 + ca*nz2;
				ny2 = tmp;
			}
			GL11.glEnd();
			start_nx = start_nx2;
			start_ny = start_ny2;
		}
	}


	// draw a cylinder of length l and radius r, aligned along the z axis

	private void drawCylinder (float l, float r, float zoffset)
	{
		int i;
		float tmp,ny,nz,a,ca,sa;
		final int n = 24;	// number of sides to the cylinder (divisible by 4)

		l *= 0.5;
		a = (float)(M_PI*2.0/n);
		sa = (float) Math.sin(a);
		ca = (float) Math.cos(a);

		// draw cylinder body
		ny=1; nz=0;		  // normal vector = (0,ny,nz)
		GL11.glBegin (GL11.GL_TRIANGLE_STRIP);
		for (i=0; i<=n; i++) {
			GL11.glNormal3d (ny,nz,0);
			GL11.glVertex3d (ny*r,nz*r,l+zoffset);
			GL11.glNormal3d (ny,nz,0);
			GL11.glVertex3d (ny*r,nz*r,-l+zoffset);
			// rotate ny,nz
			tmp = ca*ny - sa*nz;
			nz = sa*ny + ca*nz;
			ny = tmp;
		}
		GL11.glEnd();

		// draw top cap
		GL11.glShadeModel (GL11.GL_FLAT);
		ny=1; nz=0;		  // normal vector = (0,ny,nz)
		GL11.glBegin (GL11.GL_TRIANGLE_FAN);
		GL11.glNormal3d (0,0,1);
		GL11.glVertex3d (0,0,l+zoffset);
		for (i=0; i<=n; i++) {
			if (i==1 || i==n/2+1)
				setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
			GL11.glNormal3d (0,0,1);
			GL11.glVertex3d (ny*r,nz*r,l+zoffset);
			if (i==1 || i==n/2+1)
				setColor (color[0],color[1],color[2],color[3]);

			// rotate ny,nz
			tmp = ca*ny - sa*nz;
			nz = sa*ny + ca*nz;
			ny = tmp;
		}
		GL11.glEnd();

		// draw bottom cap
		ny=1; nz=0;		  // normal vector = (0,ny,nz)
		GL11.glBegin (GL11.GL_TRIANGLE_FAN);
		GL11.glNormal3d (0,0,-1);
		GL11.glVertex3d (0,0,-l+zoffset);
		for (i=0; i<=n; i++) {
			if (i==1 || i==n/2+1)
				setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
			GL11.glNormal3d (0,0,-1);
			GL11.glVertex3d (ny*r,nz*r,-l+zoffset);
			if (i==1 || i==n/2+1)
				setColor (color[0],color[1],color[2],color[3]);

			// rotate ny,nz
			tmp = ca*ny + sa*nz;
			nz = -sa*ny + ca*nz;
			ny = tmp;
		}
		GL11.glEnd();
	}

	//***************************************************************************
	// motion model

	// current camera position and orientation
	private float[] view_xyz = new float[3];	// position x,y,z
	private float[] view_hpr = new float[3];	// heading, pitch, roll (degrees)


	// initialize the above variables

	private void initMotionModel()
	{
		view_xyz[0] = 2;
		view_xyz[1] = 0;
		view_xyz[2] = 1;
		view_hpr[0] = 180;
		view_hpr[1] = 0;
		view_hpr[2] = 0;
	}


	private void wrapCameraAngles()
	{
		for (int i=0; i<3; i++) {
			while (view_hpr[i] > 180) view_hpr[i] -= 360;
			while (view_hpr[i] < -180) view_hpr[i] += 360;
		}
	}


	/**
	 * Call this to update the current camera position. the bits in `mode' say
	 * if the left (1), middle (2) or right (4) mouse button is pressed, and
	 * (deltax,deltay) is the amount by which the mouse pointer has moved.
	 */
	@Override
	void dsMotion (int mode, int deltax, int deltay)
	{
		float side = 0.01f * deltax;
		float fwd = (mode==4) ? (0.01f * deltay) : 0.0f;
		float s = (float) Math.sin (view_hpr[0]*DEG_TO_RAD);
		float c = (float) Math.cos (view_hpr[0]*DEG_TO_RAD);

		if (mode==1) {
			view_hpr[0] += deltax * 0.5f;
			view_hpr[1] += deltay * 0.5f;
		}
		else {
			view_xyz[0] += -s*side + c*fwd;
			view_xyz[1] += c*side + s*fwd;
			if (mode==2 || mode==5) view_xyz[2] += 0.01f * deltay;
		}
		wrapCameraAngles();
	}

	//***************************************************************************
	// drawing loop stuff

	// the current state:
	//    0 = uninitialized
	//    1 = dsSimulationLoop() called
	//    2 = dsDrawFrame() called
	private static int current_state = 0;

	// textures and shadows
	private static boolean use_textures=true;		// 1 if textures to be drawn
	private static boolean use_shadows=true;		// 1 if shadows to be drawn
	private static Texture sky_texture = null;
	private static Texture ground_texture = null;
	private static Texture wood_texture = null;
	private static Texture checkered_texture = null;
	private static Texture[] texture = new Texture[4+1]; // +1 since index 0 is not used



//	#ifndef macintosh

//	void dsStartGraphics (int width, int height, dsFunctions *fn)
	@Override
	void dsStartGraphics (int width, int height, dsFunctions fn)
	{

		String prefix = DEFAULT_PATH_TO_TEXTURES;
		if (fn.dsGetVersion() >= 2 && fn.dsGetPathToTextures()!=null) 
			prefix = fn.dsGetPathToTextures();
//		char *s = (char*) alloca (strlen(prefix) + 20);
		
//		strcpy (s,prefix);
//		strcat (s,"/sky.ppm");
		sky_texture = new Texture (prefix+"/sky.ppm");
		texture[DS_TEXTURE_NUMBER.DS_SKY.ordinal()] = sky_texture;

//		strcpy (s,prefix);
//		strcat (s,"/ground.ppm");
		ground_texture = new Texture (prefix+"/ground.ppm");
		texture[DS_TEXTURE_NUMBER.DS_GROUND.ordinal()] = ground_texture;

//		strcpy (s,prefix);
//		strcat (s,"/wood.ppm");
		wood_texture = new Texture (prefix+"/wood.ppm");
		texture[DS_TEXTURE_NUMBER.DS_WOOD.ordinal()] = wood_texture; 

//		strcpy (s,prefix);
//		strcat (s,"/checkered.ppm");
		checkered_texture = new Texture (prefix+"/checkered.ppm");
		texture[DS_TEXTURE_NUMBER.DS_CHECKERED.ordinal()] = checkered_texture; 
	}


	@Override
	void dsStopGraphics()
	{
//		delete sky_texture;
//		delete ground_texture;
//		delete wood_texture;
//		sky_texture = 0;
//		ground_texture = 0;
//		wood_texture = 0;
		sky_texture = null;
		ground_texture = null;
		wood_texture = null;
	}


	private static float offset = 0.0f;
//	static void drawSky (float view_xyz[3])
	private void drawSky (float[] view_xyz)
	{
		GL11.glDisable (GL11.GL_LIGHTING);
		if (use_textures) {
			GL11.glEnable (GL11.GL_TEXTURE_2D);
			sky_texture.bind (false);
		}
		else {
			GL11.glDisable (GL11.GL_TEXTURE_2D);
			GL11.glColor3f (0f,0.5f,1.0f);
		}

		// make sure sky depth is as far back as possible
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glEnable (GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc (GL11.GL_LEQUAL);
		GL11.glDepthRange (1,1);

		final float ssize = 1000.0f;

		float x = ssize*sky_scale;
		float z = view_xyz[2] + sky_height;

		GL11.glBegin (GL11.GL_QUADS);
		GL11.glNormal3f (0,0,-1);
		GL11.glTexCoord2f (-x+offset,-x+offset);
		GL11.glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
		GL11.glTexCoord2f (-x+offset,x+offset);
		GL11.glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
		GL11.glTexCoord2f (x+offset,x+offset);
		GL11.glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
		GL11.glTexCoord2f (x+offset,-x+offset);
		GL11.glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
		GL11.glEnd();

		offset = offset + 0.002f;
		if (offset > 1) offset -= 1;

		GL11.glDepthFunc (GL11.GL_LESS);
		GL11.glDepthRange (0,1);
	}


	private void drawGround()
	{
		GL11.glDisable (GL11.GL_LIGHTING);
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glEnable (GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc (GL11.GL_LESS);
		// GL11.glDepthRange (1,1);

		if (use_textures) {
			GL11.glEnable (GL11.GL_TEXTURE_2D);
			ground_texture.bind (false);
		}
		else {
			GL11.glDisable (GL11.GL_TEXTURE_2D);
			GL11.glColor3f (GROUND_R,GROUND_G,GROUND_B);
		}

		// ground fog seems to cause problems with TNT2 under windows
		/*
  GLfloat fogColor[4] = {0.5, 0.5, 0.5, 1};
  GL11.glEnable (GL_FOG);
  GL11.glFogi (GL_FOG_MODE, GL_EXP2);
  GL11.glFogfv (GL_FOG_COLOR, fogColor);
  GL11.glFogf (GL_FOG_DENSITY, 0.05f);
  GL11.glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
  GL11.glFogf (GL_FOG_START, 1.0);
  GL11.glFogf (GL_FOG_END, 5.0);
		 */

		final float gsize = 100.0f;
		final float offset = 0; // -0.001f; ... polygon offsetting doesn't work well

		GL11.glBegin (GL11.GL_QUADS);
		GL11.glNormal3f (0,0,1);
		GL11.glTexCoord2f (-gsize*ground_scale + ground_ofsx,
				-gsize*ground_scale + ground_ofsy);
		GL11.glVertex3f (-gsize,-gsize,offset);
		GL11.glTexCoord2f (gsize*ground_scale + ground_ofsx,
				-gsize*ground_scale + ground_ofsy);
		GL11.glVertex3f (gsize,-gsize,offset);
		GL11.glTexCoord2f (gsize*ground_scale + ground_ofsx,
				gsize*ground_scale + ground_ofsy);
		GL11.glVertex3f (gsize,gsize,offset);
		GL11.glTexCoord2f (-gsize*ground_scale + ground_ofsx,
				gsize*ground_scale + ground_ofsy);
		GL11.glVertex3f (-gsize,gsize,offset);
		GL11.glEnd();

		GL11.glDisable (GL11.GL_FOG);
	}


	private void drawPyramidGrid()
	{
		// setup stuff
		GL11.glEnable (GL11.GL_LIGHTING);
		GL11.glDisable (GL11.GL_TEXTURE_2D);
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glEnable (GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc (GL11.GL_LESS);

		// draw the pyramid grid
		for (int i=-1; i<=1; i++) {
			for (int j=-1; j<=1; j++) {
				GL11.glPushMatrix();
				GL11.glTranslatef (i,j,0);
				if (i==1 && j==0) setColor (1,0,0,1);
				else if (i==0 && j==1) setColor (0,0,1,1);
				else setColor (1,1,0,1);
				final float k = 0.03f;
				GL11.glBegin (GL11.GL_TRIANGLE_FAN);
				GL11.glNormal3f (0,-1,1);
				GL11.glVertex3f (0,0,k);
				GL11.glVertex3f (-k,-k,0);
				GL11.glVertex3f ( k,-k,0);
				GL11.glNormal3f (1,0,1);
				GL11.glVertex3f ( k, k,0);
				GL11.glNormal3f (0,1,1);
				GL11.glVertex3f (-k, k,0);
				GL11.glNormal3f (-1,0,1);
				GL11.glVertex3f (-k,-k,0);
				GL11.glEnd();
				GL11.glPopMatrix();
			}
		}
	}


	//static GLfloat 
	private static final FloatBuffer light_position =  BufferUtils.createFloatBuffer(4);
	private static final FloatBuffer light_ambient = BufferUtils.createFloatBuffer(4);
	private static final FloatBuffer light_diffuse = BufferUtils.createFloatBuffer(4);
	private static final FloatBuffer light_specular = BufferUtils.createFloatBuffer(4);
	static {
		light_position.put(new float[] { LIGHTX, LIGHTY, 1.0f, 0.0f }).flip();
		light_ambient.put(new float[]{ 0.5f, 0.5f, 0.5f, 1.0f }).flip();
		light_diffuse.put(new float[] { 1.0f, 1.0f, 1.0f, 1.0f }).flip();
		light_specular.put(new float[] { 1.0f, 1.0f, 1.0f, 1.0f }).flip();
	}

	@Override
	//	void dsDrawFrame (int width, int height, dsFunctions *fn, int pause)
	void dsDrawFrame (int width, int height, dsFunctions fn, boolean pause)
	{
		if (current_state < 1) dsDebug ("internal error");
		current_state = 2;

		// setup stuff
		GL11.glEnable (GL11.GL_LIGHTING);
		GL11.glEnable (GL11.GL_LIGHT0);
		GL11.glDisable (GL11.GL_TEXTURE_2D);
		GL11.glDisable (GL11.GL_TEXTURE_GEN_S);
		GL11.glDisable (GL11.GL_TEXTURE_GEN_T);
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glEnable (GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc (GL11.GL_LESS);
		GL11.glEnable (GL11.GL_CULL_FACE);
		GL11.glCullFace (GL11.GL_BACK);
		GL11.glFrontFace (GL11.GL_CCW);

		// setup viewport
		GL11.glViewport (0,0,width,height);
		GL11.glMatrixMode (GL11.GL_PROJECTION);
		GL11.glLoadIdentity();
		final float vnear = 0.1f;
		final float vfar = 100.0f;
		final float k = 0.8f;     // view scale, 1 = +/- 45 degrees
		if (width >= height) {
			float k2 = (float)height/(float)width;
			GL11.glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
		}
		else {
			float k2 = (float)width/(float)height;
			GL11.glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
		}
		
		// setup lights. it makes a difference whether this is done in the
		// GL_PROJECTION matrix mode (lights are scene relative) or the
		// GL_MODELVIEW matrix mode (lights are camera relative, bad!).
//		static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
//		static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
//		static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
		GL11.glLightfv (GL11.GL_LIGHT0, GL11.GL_AMBIENT, light_ambient);
		GL11.glLightfv (GL11.GL_LIGHT0, GL11.GL_DIFFUSE, light_diffuse);
		GL11.glLightfv (GL11.GL_LIGHT0, GL11.GL_SPECULAR, light_specular);
		GL11.glColor3f (1.0f, 1.0f, 1.0f);

		// clear the window
		GL11.glClearColor (0.5f ,0.5f ,0.5f ,0);
		GL11.glClear (GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);

		// snapshot camera position (in MS Windows it is changed by the GUI thread)
		float[] view2_xyz=view_xyz.clone();
		float[] view2_hpr=view_hpr.clone();
//		memcpy (view2_xyz,view_xyz);//,sizeof(float)*3);
//		memcpy (view2_hpr,view_hpr);//,sizeof(float)*3);

		// go to GL_MODELVIEW matrix mode and set the camera
		GL11.glMatrixMode (GL11.GL_MODELVIEW);
		GL11.glLoadIdentity();
		setCamera (view2_xyz[0],view2_xyz[1],view2_xyz[2],
				view2_hpr[0],view2_hpr[1],view2_hpr[2]);

		// set the light position (for some reason we have to do this in model view.
//		static GLfloat light_position[] = { LIGHTX, LIGHTY, 1.0, 0.0 };
		GL11.glLightfv (GL11.GL_LIGHT0, GL11.GL_POSITION, light_position);

		// draw the background (ground, sky etc)
		drawSky (view2_xyz);
		drawGround();

		// draw the little markers on the ground
		drawPyramidGrid();

		// leave openGL in a known state - flat shaded white, no textures
		GL11.glEnable (GL11.GL_LIGHTING);
		GL11.glDisable (GL11.GL_TEXTURE_2D);
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glEnable (GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc (GL11.GL_LESS);
		GL11.glColor3f (1,1,1);
		setColor (1,1,1,1);

		// draw the rest of the objects. set drawing state first.
		color[0] = 1;
		color[1] = 1;
		color[2] = 1;
		color[3] = 1;
		tnum = DS_TEXTURE_NUMBER.DS_NONE;
		//if (fn.step) 
		fn.step(pause);
	}


	@Override
	boolean dsGetShadows()
	{
		return use_shadows;
	}


	@Override
	void dsSetShadows (boolean a) {
		use_shadows = a;
	}


	@Override
	boolean dsGetTextures()
	{
		return use_textures;
	}


	@Override
	void dsSetTextures (boolean a) {
		use_textures = a;
	}

	//***************************************************************************
	// C interface

	// sets lighting and texture modes, sets current color
	private static final FloatBuffer s_params_SDM = BufferUtils.createFloatBuffer(4);
	private static final FloatBuffer t_params_SDM = BufferUtils.createFloatBuffer(4);
	static {
		s_params_SDM.put(new float[]{1.0f,1.0f,0.0f,1}).flip();
		t_params_SDM.put(new float[]{0.817f,-0.817f,0.817f,1}).flip();
	}
	private void setupDrawingMode()
	{
		GL11.glEnable (GL11.GL_LIGHTING);
		if (tnum != DS_TEXTURE_NUMBER.DS_NONE) {
			if (use_textures) {
				GL11.glEnable (GL11.GL_TEXTURE_2D);
				texture[tnum.ordinal()].bind (true);
				GL11.glEnable (GL11.GL_TEXTURE_GEN_S);
				GL11.glEnable (GL11.GL_TEXTURE_GEN_T);
				GL11.glTexGeni (GL11.GL_S,GL11.GL_TEXTURE_GEN_MODE,GL11.GL_OBJECT_LINEAR);
				GL11.glTexGeni (GL11.GL_T,GL11.GL_TEXTURE_GEN_MODE,GL11.GL_OBJECT_LINEAR);
//				static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
//				static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
				GL11.glTexGenfv (GL11.GL_S,GL11.GL_OBJECT_PLANE,s_params_SDM);
				GL11.glTexGenfv (GL11.GL_T,GL11.GL_OBJECT_PLANE,t_params_SDM);
			}
			else {
				GL11.glDisable (GL11.GL_TEXTURE_2D);
			}
		}
		else {
			GL11.glDisable (GL11.GL_TEXTURE_2D);
		}
		setColor (color[0],color[1],color[2],color[3]);

		if (color[3] < 1) {
			GL11.glEnable (GL11.GL_BLEND);
			GL11.glBlendFunc (GL11.GL_SRC_ALPHA,GL11.GL_ONE_MINUS_SRC_ALPHA);
		}
		else {
			GL11.glDisable (GL11.GL_BLEND);
		}
	}


	private static final FloatBuffer s_params_SSDM = BufferUtils.createFloatBuffer(4);
	private static final FloatBuffer t_params_SSDM = BufferUtils.createFloatBuffer(4);
	static {
		s_params_SSDM.put(new float[]{ground_scale,0,0,ground_ofsx}).flip();
		t_params_SSDM.put(new float[]{0,ground_scale,0,ground_ofsy}).flip();
	}
	private void setShadowDrawingMode()
	{
		GL11.glDisable (GL11.GL_LIGHTING);
		if (use_textures) {
			GL11.glEnable (GL11.GL_TEXTURE_2D);
			ground_texture.bind (true);
			GL11.glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
			GL11.glEnable (GL11.GL_TEXTURE_2D);
			GL11.glEnable (GL11.GL_TEXTURE_GEN_S);
			GL11.glEnable (GL11.GL_TEXTURE_GEN_T);
			GL11.glTexGeni (GL11.GL_S,GL11.GL_TEXTURE_GEN_MODE,GL11.GL_EYE_LINEAR);
			GL11.glTexGeni (GL11.GL_T,GL11.GL_TEXTURE_GEN_MODE,GL11.GL_EYE_LINEAR);
//			static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
//			static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
			GL11.glTexGenfv (GL11.GL_S,GL11.GL_EYE_PLANE,s_params_SSDM);
			GL11.glTexGenfv (GL11.GL_T,GL11.GL_EYE_PLANE,t_params_SSDM);
		}
		else {
			GL11.glDisable (GL11.GL_TEXTURE_2D);
			GL11.glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
					GROUND_B*SHADOW_INTENSITY);
		}
		GL11.glDepthRange (0,0.9999);
	}


	
	//extern "C" 
	/**
	 * If you filter out arguments beforehand, simply set them to "".
	 * @see org.ode4j.drawstuff.DrawStuff#dsSimulationLoop(String[], int, int, dsFunctions)
	 */
	@Override
	public void dsSimulationLoop (String[] args,
			int window_width, int window_height,
			dsFunctions fn)
	{
		if (current_state != 0) dsError ("dsSimulationLoop() called more than once");
		current_state = 1;

		// look for flags that apply to us
		boolean initial_pause = false;
		for (int i=0; i<args.length; i++) {
			//Ignore empty arguments
		    if (args[i] == null || args[i].equals("")) continue;
			if (args[i].equals("-h")) { fn.dsPrintHelp(); continue; }
			if (args[i].equals("-help")) { fn.dsPrintHelp(); continue; }
			if (args[i].equals("-notex")) { use_textures = false; continue; }
			if (args[i].equals("-noshadow")) {use_shadows = false; continue; }
			if (args[i].equals("-noshadows")) { use_shadows = false; continue; }
			if (args[i].equals("-pause")) { initial_pause = true; continue; }
			if (args[i].equals("-texturepath"))
				if (++i < args.length) {
					fn.dsSetPathToTextures( args[i] );
					continue; 
				}
		    System.out.println("Argument not understood: \"" + args[i] + "\"");
		    fn.dsPrintHelp();
		    return;
		}

		if (fn.dsGetVersion() > DS_VERSION)
			dsDebug ("bad version number in dsFunctions structure");

		initMotionModel();
		dsPlatformSimLoop (window_width,window_height,fn,initial_pause);

		current_state = 0;
	}


	//extern "C" 
	//void dsSetViewpoint (float xyz[3], float hpr[3])
	@Override
	public void dsSetViewpoint (float[] xyz, float[] hpr)
	{
		if (current_state < 1) dsError ("dsSetViewpoint() called before simulation started");
		if (xyz!=null) {
			view_xyz[0] = xyz[0];
			view_xyz[1] = xyz[1];
			view_xyz[2] = xyz[2];
		}
		if (hpr!=null) {
			view_hpr[0] = hpr[0];
			view_hpr[1] = hpr[1];
			view_hpr[2] = hpr[2];
			wrapCameraAngles();
		}
	}


	//extern "C" 
	//void dsGetViewpoint (float xyz[3], float hpr[3])
	@Override
	public void dsGetViewpoint (float[] xyz, float[] hpr)
	{
		if (current_state < 1) dsError ("dsGetViewpoint() called before simulation started");
		if (xyz!=null) {
			xyz[0] = view_xyz[0];
			xyz[1] = view_xyz[1];
			xyz[2] = view_xyz[2];
		}
		if (hpr!=null) {
			hpr[0] = view_hpr[0];
			hpr[1] = view_hpr[1];
			hpr[2] = view_hpr[2];
		}
	}


	//extern "C" 
	/* (non-Javadoc)
	 * @see org.ode4j.drawstuff.internal.DrawStuff#dsSetTexture(org.ode4j.drawstuff.DS_API.DS_TEXTURE_NUMBER)
	 */
	@Override
	public void dsSetTexture (DS_TEXTURE_NUMBER texture_number)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		tnum = texture_number;
	}


	//extern "C" 
	/* (non-Javadoc)
	 * @see org.ode4j.drawstuff.internal.DrawStuff#dsSetColor(float, float, float)
	 */
	@Override
	public void dsSetColor (float red, float green, float blue)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		color[0] = red;
		color[1] = green;
		color[2] = blue;
		color[3] = 1;
	}


	//extern "C" 
	@Override
	public void dsSetColorAlpha (float red, float green, float blue, float alpha)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		color[0] = red;
		color[1] = green;
		color[2] = blue;
		color[3] = alpha;
	}


	//extern "C" 
//	void dsDrawBox (final float pos[3], final float R[12],
//			final float sides[3])
	/* (non-Javadoc)
	 * @see org.ode4j.drawstuff.internal.DrawStuff#dsDrawBox(float[], float[], float[])
	 */
	@Override
	public void dsDrawBox (final float[] pos, final float[] R,
			final float[] sides)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawBox (sides);
		GL11.glPopMatrix();

		if (use_shadows) {
			setShadowDrawingMode();
			setShadowTransform();
			setTransform (pos,R);
			drawBox (sides);
			GL11.glPopMatrix();
			GL11.glPopMatrix();
			GL11.glDepthRange (0,1);
		}
	}

	//extern "C" 
//	void dsDrawConvex (final float pos[3], final float R[12],
//			float *_planes,unsigned int _planecount,
//			float *_points,
//			unsigned int _pointcount,
//			unsigned int *_polygons)
	void dsDrawConvex (final float[] pos, final float[] R,
			float[] _planes, int _planecount,
			float[] _points,
			int _pointcount,
			int[] _polygons)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawConvex(_planes,_planecount,_points,_pointcount,_polygons);
		GL11.glPopMatrix();
		if (use_shadows) {
			setShadowDrawingMode();
			setShadowTransform();
			setTransform (pos,R);
			drawConvex(_planes,_planecount,_points,_pointcount,_polygons);
			GL11.glPopMatrix();
			GL11.glPopMatrix();
			GL11.glDepthRange (0,1);
		}
	}


	//extern "C" 
//	void dsDrawSphere (final float pos[3], final float R[12],
//			float radius)
	/* (non-Javadoc)
	 * @see org.ode4j.drawstuff.internal.DrawStuff#dsDrawSphere(float[], float[], float)
	 */
	@Override
	public void dsDrawSphere (final float[] pos, final float[] R, float radius)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glEnable (GL11.GL_NORMALIZE);
		GL11.glShadeModel (GL11.GL_SMOOTH);
		setTransform (pos,R);
		GL11.glScaled (radius,radius,radius);
		drawSphere();
		GL11.glPopMatrix();
		GL11.glDisable (GL11.GL_NORMALIZE);

		// draw shadows
		if (use_shadows) {
			GL11.glDisable (GL11.GL_LIGHTING);
			if (use_textures) {
				ground_texture.bind (true);
				GL11.glEnable (GL11.GL_TEXTURE_2D);
				GL11.glDisable (GL11.GL_TEXTURE_GEN_S);
				GL11.glDisable (GL11.GL_TEXTURE_GEN_T);
				GL11.glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
			}
			else {
				GL11.glDisable (GL11.GL_TEXTURE_2D);
				GL11.glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
						GROUND_B*SHADOW_INTENSITY);
			}
			GL11.glShadeModel (GL11.GL_FLAT);
			GL11.glDepthRange (0,0.9999);
			drawSphereShadow (pos[0],pos[1],pos[2],radius);
			GL11.glDepthRange (0,1);
		}
	}


	//extern "C" 
//	void dsDrawTriangle (final float pos[3], final float R[12],
//			final float *v0, final float *v1,
//			final float *v2, int solid)
	void dsDrawTriangle (final float[] pos, final float[] R,
			final float[] v0, final float[] v1,
			final float[] v2, boolean solid)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawTriangle (v0, v1, v2, solid);
		GL11.glPopMatrix();
	}


	void dsDrawTriangle (final float[] pos, final float[] R,
			final float[] vAll, final int v0, final int v1,
			final int v2, boolean solid)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawTriangle (vAll, v0, v1, v2, solid);
		GL11.glPopMatrix();
	}


	@Override
	public void dsDrawTriangle (final DVector3C pos, final DMatrix3C R,
			final float[] vAll, final int v0, final int v1,
			final int v2, boolean solid)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawTriangle (vAll, v0, v1, v2, solid);
		GL11.glPopMatrix();
	}


	@Override
	public void dsDrawTriangle (final DVector3C pos, final DMatrix3C R,
			final float[] v0, final float[] v1, final float[] v2, boolean solid)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawTriangle (v0, v1, v2, solid);
		GL11.glPopMatrix();
	}

	//	extern "C" void dsDrawTriangles (const float pos[3], const float R[12],
//				const float *v, int n, int solid)
	@Override
	public void dsDrawTriangles(final float[] pos, final float[] R,
								final float[][] v, boolean solid) {
		if (current_state != 2) dsError("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel(GL11.GL_FLAT);
		setTransform(pos, R);
		int n = v.length / 3;
		for (int i = 0; i < n; ++i) {
			drawTriangle(v[3 * i], v[3 * i + 1], v[3 * i + 2], solid);
		}
		GL11.glPopMatrix();
	}

	//extern "C" 
//	void dsDrawCylinder (final float pos[3], final float R[12],
//			float length, float radius)
	/**
	 * @see DrawStuffApi#dsDrawCylinder(float[], float[], float, float)
	 */
	@Override
	public void dsDrawCylinder (final float[] pos, final float[] R,
			float length, float radius)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_SMOOTH);
		setTransform (pos,R);
		drawCylinder (length,radius,0);
		GL11.glPopMatrix();

		if (use_shadows) {
			setShadowDrawingMode();
			setShadowTransform();
			setTransform (pos,R);
			drawCylinder (length,radius,0);
			GL11.glPopMatrix();
			GL11.glPopMatrix();
			GL11.glDepthRange (0,1);
		}
	}


	//extern "C" 
//	void dsDrawCapsule (final float pos[3], final float R[12],
//			float length, float radius)
	/**
	 * @see DrawStuffApi#dsDrawCapsule(float[], float[], float, float)
	 */
	@Override
	public void dsDrawCapsule (final float[] pos, final float[] R,
			float length, float radius)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_SMOOTH);
		setTransform (pos,R);
		drawCapsule (length,radius);
		GL11.glPopMatrix();

		if (use_shadows) {
			setShadowDrawingMode();
			setShadowTransform();
			setTransform (pos,R);
			drawCapsule (length,radius);
			GL11.glPopMatrix();
			GL11.glPopMatrix();
			GL11.glDepthRange (0,1);
		}
	}


//	void dsDrawLine (final float pos1[3], final float pos2[3])
	/** 
	 * @see DrawStuffApi#dsDrawLine(float[], float[])
	 */
	@Override
	public void dsDrawLine (final float[] pos1, final float[] pos2)
	{
		setupDrawingMode();
		GL11.glColor3f (color[0],color[1],color[2]);
		GL11.glDisable (GL11.GL_LIGHTING);
		GL11.glLineWidth (2);
		GL11.glShadeModel (GL11.GL_FLAT);
		GL11.glBegin (GL11.GL_LINES);
		GL11.glVertex3f (pos1[0],pos1[1],pos1[2]);
		GL11.glVertex3f (pos2[0],pos2[1],pos2[2]);
		GL11.glEnd();
	}


//	void dsDrawBoxD (final double pos[3], final double R[12],
//			final double sides[3])
	/**
	 * @see DrawStuffApi#dsDrawBox(float[], float[], float[])
	 */
	@Override
	public void dsDrawBox (DVector3C pos, DMatrix3C R, DVector3C sides)
	{
		float[] pos2=pos.toFloatArray4();
		float[] R2=R.toFloatArray12();
		float[] fsides=sides.toFloatArray4();
		dsDrawBox (pos2,R2,fsides);
	}

	//extern "C" 
//	void dsDrawConvexD (final double pos[3], final double R[12],
//			double *_planes,unsigned int _planecount,
//			double *_points,
//			unsigned int _pointcount,
//			unsigned int *_polygons)
	/**
	 * @see DrawStuffApi#dsDrawConvex(DVector3C, DMatrix3C, double[], int, double[], int, int[])
	 */
	@Override
	public void dsDrawConvex (DVector3C pos, DMatrix3C R,
			double[] _planes, int _planecount,
			double[] _points,
			int _pointcount,
			int[] _polygons)
	{
		if (current_state != 2) dsError ("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawConvexD(_planes,_planecount,_points,_pointcount,_polygons);
		GL11.glPopMatrix();
		if (use_shadows) {
			setShadowDrawingMode();
			setShadowTransform();
			setTransform (pos,R);
			drawConvexD(_planes,_planecount,_points,_pointcount,_polygons);
			GL11.glPopMatrix();
			GL11.glPopMatrix();
			GL11.glDepthRange (0,1);
		}
	}

//	void dsDrawSphereD (final double pos[3], final double R[12], float radius)
	/** 
	 * @see DrawStuffApi#dsDrawSphere(DVector3C, DMatrix3C, float)
	 */
	@Override
	public void dsDrawSphere (final DVector3C pos, final DMatrix3C R, float radius)
	{
		float[] pos2=pos.toFloatArray4();
		float[] R2=R.toFloatArray12();
		dsDrawSphere (pos2,R2,radius);
	}


//	void dsDrawTriangleD (final double pos[3], final double R[12],
//			final double *v0, final double *v1,
//			final double *v2, int solid)
//	void dsDrawTriangleD (final double[] pos, final double[] R,
//			final double[] v0, final double[] v1,
//			final double[] v2, boolean solid)
//	{
//		int i;
//		float[] pos2=new float[3],R2=new float[12];
//		for (i=0; i<3; i++) pos2[i]=(float)pos[i];
//		for (i=0; i<12; i++) R2[i]=(float)R[i];
//
//		setupDrawingMode();
//		GL11.glShadeModel (GL11.GL_FLAT);
//		setTransform (pos2,R2);
//		drawTriangleD (v0, v1, v2, solid);
//		GL11.glPopMatrix();
//	}


	@Override
	public void dsDrawTriangle (final DVector3C pos, final DMatrix3C R,
			final DVector3C v0, final DVector3C v1,
			final DVector3C v2, boolean solid)
	{
		setupDrawingMode();
		GL11.glShadeModel (GL11.GL_FLAT);
		setTransform (pos,R);
		drawTriangle (v0, v1, v2, solid);
		GL11.glPopMatrix();
	}

	//	extern "C" void dsDrawTrianglesD (const double pos[3], const double R[12],
//				const double *v, int n, int solid)
	public void dsDrawTriangles(final DVector3C pos, final DMatrix3C R,
								final DVector3C[] v, boolean solid) {
		int i;
		DVector3C pos2 = new DVector3(pos);
		DMatrix3C R2 = new DMatrix3(R);
//		for (i=0; i<3; i++) pos2[i]=(float)pos[i];
//		for (i=0; i<12; i++) R2[i]=(float)R[i];

		if (current_state != 2) dsError("drawing function called outside simulation loop");
		setupDrawingMode();
		GL11.glShadeModel(GL11.GL_FLAT);
		setTransform(pos2, R2);
		for (i = 0; i < v.length; ++i)
			drawTriangle(v[3 * i], v[3 * i + 1], v[3 * i + 2], solid);
		GL11.glPopMatrix();
	}


//	void dsDrawCylinderD (final double pos[3], final double R[12],
//			float length, float radius)
	/**
	 * @see DrawStuffApi#dsDrawCylinder(DVector3C, DMatrix3C, float, float)
	 */
	@Override
	public void dsDrawCylinder (final DVector3C pos, final DMatrix3C R,
			float length, float radius)
	{
		float[] pos2=pos.toFloatArray4();
		float[] R2=R.toFloatArray12();
		dsDrawCylinder (pos2,R2,length,radius);
	}


//	void dsDrawCapsuleD (final double pos[3], final double R[12],
//			float length, float radius)
	/**
	 * @see DrawStuffApi#dsDrawCapsule(DVector3C, DMatrix3C, float, float)
	 */
	@Override
	public void dsDrawCapsule (final DVector3C pos, final DMatrix3C R,
			float length, float radius)
	{
		float[] pos2=pos.toFloatArray4();
		float[] R2=R.toFloatArray12();
		dsDrawCapsule (pos2,R2,length,radius);
	}


//	void dsDrawLineD (final double _pos1[3], final double _pos2[3])
	/**
	 * @see DrawStuffApi#dsDrawLine(DVector3C, DVector3C)
	 */
	@Override
	public void dsDrawLine (final DVector3C pos1, final DVector3C pos2)
	{
		float[] pos1f=pos1.toFloatArray4();
		float[] pos2f=pos2.toFloatArray4();
		dsDrawLine (pos1f,pos2f);
	}


	@Override
	public void dsSetSphereQuality (int n)
	{
		sphere_quality = n;
	}


	void dsSetCapsuleQuality (int n)
	{
		capped_cylinder_quality = n;
	}

	@Override
	public void dsSetDrawMode(int mode)
	{
		switch(mode)
		{
		case DS_POLYFILL:
			GL11.glPolygonMode(GL11.GL_FRONT,GL11.GL_FILL);
			break;
		case DS_WIREFRAME:
			GL11.glPolygonMode(GL11.GL_FRONT,GL11.GL_LINE);
			break;
		}
	}
}
