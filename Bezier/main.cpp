#include <Windows.h>
#include "Rig3D\Engine.h"
#include "Rig3D\Graphics\Interface\IScene.h"
#include "Rig3D\Graphics\DirectX11\DX3D11Renderer.h"
#include "Rig3D\Graphics\Interface\IMesh.h"
#include "Rig3D\Graphics\DirectX11\DX11Mesh.h"
#include "Rig3D\Common\Transform.h"
#include "Memory\Memory\LinearAllocator.h"
#include "Rig3D\MeshLibrary.h"
#include <d3d11.h>
#include <d3dcompiler.h>
#include <fstream>
#include <fvec.h>

#define PI 3.1415926535f

#define SHUFFLE_PARAM(x, y, z, w) ((x) | ((y) << 2) | ((z) << 4) | ((w) << 6))
#define _mm_replicate_x_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(0, 0, 0, 0))
#define _mm_replicate_y_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(1, 1, 1, 1))
#define _mm_replicate_z_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(2, 2, 2, 2))
#define _mm_replicate_w_ps(v) _mm_shuffle_ps((v), (v), SHUFFLE_PARAM(3, 3, 3, 3))
#define _mm_add_mul_ps(a, b, c) _mm_add_ps(_mm_mul_ps((a), (b)), (c))

#if defined(_MSC_VER)
#define ALIGNED(x) __declspec(align(x))
// other Alignment here..
#endif

using namespace Rig3D;

// cubic degree bezier curve
class Bezier
{
public:
	union
	{
		struct
		{
			vec4f p0;
			vec4f p1;
			vec4f p2;
			vec4f p3;
		};
		mat4f p;
	};

	Bezier() { }
	~Bezier() { }

	// Optimized SISD Cubic Bezier equation
	// Equation based on this article:
	// http://www.idav.ucdavis.edu/education/CAGDNotes/Matrix-Cubic-Bezier-Curve/Matrix-Cubic-Bezier-Curve.html
	void Evaluate(const float time, vec4f* result)
	{
		static mat4f m = mat4f(
			 1,  0,  0,  0,
			-3,  3,  0,  0,
			 3, -6,  3,  0, 
			-1,  3, -3,  1);

		vec4f t = vec4f(1.0f, time, time*time, time*time*time);

		auto m0 = m.u;
		auto m1 = m.v;
		auto m2 = m.w;
		auto m3 = m.t;

		vec4f tM = vec4f(
			t.data[0] * m0.data[0] + t.data[1] * m1.data[0] + t.data[2] * m2.data[0] + t.data[3] * m3.data[0],
			t.data[0] * m0.data[1] + t.data[1] * m1.data[1] + t.data[2] * m2.data[1] + t.data[3] * m3.data[1],
			t.data[0] * m0.data[2] + t.data[1] * m1.data[2] + t.data[2] * m2.data[2] + t.data[3] * m3.data[2],
			t.data[0] * m0.data[3] + t.data[1] * m1.data[3] + t.data[2] * m2.data[3] + t.data[3] * m3.data[3]
		);

		(*result).data[0] = tM.data[0] * p0.data[0] + tM.data[1] * p1.data[0] + tM.data[2] * p2.data[0] + tM.data[3] * p3.data[0];
		(*result).data[1] = tM.data[0] * p0.data[1] + tM.data[1] * p1.data[1] + tM.data[2] * p2.data[1] + tM.data[3] * p3.data[1];
		(*result).data[2] = tM.data[0] * p0.data[2] + tM.data[1] * p1.data[2] + tM.data[2] * p2.data[2] + tM.data[3] * p3.data[2];
		(*result).data[3] = tM.data[0] * p0.data[3] + tM.data[1] * p1.data[3] + tM.data[2] * p2.data[3] + tM.data[3] * p3.data[3];
	}

	// Optimized SIMD Cubic Bezier Equation
	// Equation based on this article:
	// http://www.idav.ucdavis.edu/education/CAGDNotes/Matrix-Cubic-Bezier-Curve/Matrix-Cubic-Bezier-Curve.html
	void EvaluateSIMD(const float time, vec4f* result)
	{
		static mat4f m = mat4f(
			 1,  0,  0,  0,
			-3,  3,  0,  0,
			 3, -6,  3,  0, 
			-1,  3, -3,  1);

		// (1, t, t^2, t^3)
		__m128 t = _mm_set_ps(time*time*time, time*time, time, 1);

		__m128 a = _mm_load_ps(m.u.data);

		// Vector Matrix multiplication between t and M.
		__m128 tM = _mm_mul_ps(_mm_replicate_x_ps(t), _mm_load_ps(m.u.data));
		tM = _mm_add_mul_ps(_mm_replicate_y_ps(t), _mm_load_ps(m.v.data), tM);
		tM = _mm_add_mul_ps(_mm_replicate_z_ps(t), _mm_load_ps(m.w.data), tM);
		tM = _mm_add_mul_ps(_mm_replicate_w_ps(t), _mm_load_ps(m.t.data), tM);

		// Vector Matrix multiplication between the result matrix of t*M and 
		// the matrix P formed by the control points (p0 ... p3)
		__m128 tMP = _mm_mul_ps(_mm_replicate_x_ps(tM), _mm_load_ps(p0.data));
		tMP = _mm_add_mul_ps(_mm_replicate_y_ps(tM), _mm_load_ps(p1.data), tMP);
		tMP = _mm_add_mul_ps(_mm_replicate_z_ps(tM), _mm_load_ps(p2.data), tMP);
		tMP = _mm_add_mul_ps(_mm_replicate_w_ps(tM), _mm_load_ps(p3.data), tMP);

		_mm_store_ps(result->data, tMP);
	}
};

static const int BEZIER_VERTEX_COUNT = 100;
static const int BEZIER_INDEX_COUNT = (BEZIER_VERTEX_COUNT - 1) * 2;

static const int HANDLES_VERTEX_COUNT = 4;
static const int HANDLES_INDEX_COUNT = 4;

static const int CIRCLE_VERTEX_COUNT = 101;
static const int CIRCLE_INDEX_COUNT = 300;

class Rig3DSampleScene : public IScene, public virtual IRendererDelegate
{
public:

	typedef cliqCity::memory::LinearAllocator LinearAllocator;

	struct BezierVertex
	{
		vec3f mPosition;
		vec3f mColor;
	};

	struct BezierMatrixBuffer
	{
		mat4f mWorld;
		mat4f mProjection;
	};
	
	Bezier 					mBezier;
	BezierMatrixBuffer		mMatrixBuffer;

	LinearAllocator			mAllocator;
	MeshLibrary<LinearAllocator> mMeshLibrary;
	
	IMesh*					mBezierMesh;
	IMesh*					mCircleMesh;
	IMesh*					mHandlesMesh;

	BezierVertex			mBezierVertices[BEZIER_VERTEX_COUNT];
	BezierVertex			mHandlesVertices[HANDLES_VERTEX_COUNT];
	
	DX3D11Renderer*			mRenderer;
	ID3D11Device*			mDevice;
	ID3D11DeviceContext*	mDeviceContext;

	ID3D11Buffer*			mConstantBuffer;
	ID3D11InputLayout*		mInputLayout;
	ID3D11VertexShader*		mVertexShader;
	ID3D11PixelShader*		mPixelShader;

	Rig3DSampleScene() : mAllocator(1024)
	{
		mOptions.mWindowCaption = "SIMD Bezier";
		mOptions.mWindowWidth = 800;
		mOptions.mWindowHeight = 600;
		mOptions.mGraphicsAPI = GRAPHICS_API_DIRECTX11;
		mOptions.mFullScreen = false;
		mMeshLibrary.SetAllocator(&mAllocator);
		size_t a = alignof(Bezier);
	}

	~Rig3DSampleScene()
	{
		ReleaseMacro(mVertexShader);
		ReleaseMacro(mPixelShader);
		ReleaseMacro(mConstantBuffer);
		ReleaseMacro(mInputLayout);
	}

	void VInitialize() override
	{
		mRenderer = &DX3D11Renderer::SharedInstance();
		mRenderer->SetDelegate(this);

		mDevice = mRenderer->GetDevice();
		mDeviceContext = mRenderer->GetDeviceContext();

		mBezier.p0 = vec4f(-4, -4, 0, 0);
		mBezier.p1 = vec4f(-4, 4, 0, 0);
		mBezier.p2 = vec4f(4, -4, 0, 0);
		mBezier.p3 = vec4f(4, 4, 0, 0);

		VOnResize();

		InitializeGeometry();
		InitializeShaders();
		InitializeCamera();
	}

	void InitializeGeometry()
	{
		// ---- Bezier

		// allocate a sizeof BEZIER_VERTEX_COUNT * 2 to simplify our upcoming loop.
		uint16_t bezierIndices[BEZIER_VERTEX_COUNT * 2];

		for (size_t i = 0, j = 0; i < BEZIER_VERTEX_COUNT; i++, j += 2)
		{
			mBezierVertices[i].mColor = { 1.0f, 1.0f, 0.0f };
			mBezierVertices[i].mPosition = vec3f();

			bezierIndices[j] = i;
			bezierIndices[j + 1] = i + 1;
		}

		mMeshLibrary.NewMesh(&mBezierMesh, mRenderer);
		mRenderer->VSetMeshVertexBufferData(mBezierMesh, mBezierVertices, sizeof(BezierVertex) * BEZIER_VERTEX_COUNT, sizeof(BezierVertex), GPU_MEMORY_USAGE_DEFAULT);
		mRenderer->VSetMeshIndexBufferData(mBezierMesh, bezierIndices, BEZIER_INDEX_COUNT, GPU_MEMORY_USAGE_DEFAULT);

		// -- Handles

		uint16_t handlesIndices[HANDLES_INDEX_COUNT];

		for (size_t i = 0; i < HANDLES_VERTEX_COUNT; i++)
		{
			mHandlesVertices[i].mColor = { 0.5f, 0.5f, 0.5f };
			mHandlesVertices[i].mPosition = vec3f(1);

			handlesIndices[i] = i;
		}

		mMeshLibrary.NewMesh(&mHandlesMesh, mRenderer);
		mRenderer->VSetMeshVertexBufferData(mHandlesMesh, mHandlesVertices, sizeof(BezierVertex) * HANDLES_VERTEX_COUNT, sizeof(BezierVertex), GPU_MEMORY_USAGE_DEFAULT);
		mRenderer->VSetMeshIndexBufferData(mHandlesMesh, handlesIndices, HANDLES_INDEX_COUNT, GPU_MEMORY_USAGE_DEFAULT);

		// -- Circle

		BezierVertex circleVertices[CIRCLE_VERTEX_COUNT];
		uint16_t circleIndices[CIRCLE_INDEX_COUNT];

		circleVertices[0].mColor = { 0.6f, 0.6f, 0.6f };
		circleVertices[0].mPosition = vec3f();

		float t;
		auto index = 0;
		for (size_t i = 1; i < CIRCLE_VERTEX_COUNT; i++)
		{
			t = -static_cast<float>(i) * 2 * PI / (CIRCLE_VERTEX_COUNT - 1);
			
			circleVertices[i].mColor = { 0.6f, 0.6f, 0.6f };
			circleVertices[i].mPosition = { cos(t), sin(t), 0.0f };

			circleIndices[index++] = 0;
			circleIndices[index++] = i;
			circleIndices[index++] = i + 1;
		}

		circleIndices[index - 1] = 1;

		mMeshLibrary.NewMesh(&mCircleMesh, mRenderer);
		mRenderer->VSetMeshVertexBufferData(mCircleMesh, circleVertices, sizeof(BezierVertex) * CIRCLE_VERTEX_COUNT, sizeof(BezierVertex), GPU_MEMORY_USAGE_DEFAULT);
		mRenderer->VSetMeshIndexBufferData(mCircleMesh, circleIndices, CIRCLE_INDEX_COUNT, GPU_MEMORY_USAGE_DEFAULT);
	}

	void InitializeShaders()
	{
		D3D11_INPUT_ELEMENT_DESC inputDescription[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};

		// Load Vertex Shader --------------------------------------
		ID3DBlob* vsBlob;
		D3DReadFileToBlob(L"SampleVertexShader.cso", &vsBlob);

		// Create the shader on the device
		mDevice->CreateVertexShader(
			vsBlob->GetBufferPointer(),
			vsBlob->GetBufferSize(),
			NULL,
			&mVertexShader);

		// Before cleaning up the data, create the input layout
		if (inputDescription) {
			if (mInputLayout != NULL) ReleaseMacro(mInputLayout);
			mDevice->CreateInputLayout(
				inputDescription,					// Reference to Description
				2,									// Number of elments inside of Description
				vsBlob->GetBufferPointer(),
				vsBlob->GetBufferSize(),
				&mInputLayout);
		}

		// Clean up
		vsBlob->Release();

		// Load Pixel Shader ---------------------------------------
		ID3DBlob* psBlob;
		D3DReadFileToBlob(L"SamplePixelShader.cso", &psBlob);

		// Create the shader on the device
		mDevice->CreatePixelShader(
			psBlob->GetBufferPointer(),
			psBlob->GetBufferSize(),
			NULL,
			&mPixelShader);

		// Clean up
		psBlob->Release();

		// Constant buffers ----------------------------------------
		D3D11_BUFFER_DESC cBufferTransformDesc;
		cBufferTransformDesc.ByteWidth = sizeof(mMatrixBuffer);
		cBufferTransformDesc.Usage = D3D11_USAGE_DEFAULT;
		cBufferTransformDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		cBufferTransformDesc.CPUAccessFlags = 0;
		cBufferTransformDesc.MiscFlags = 0;
		cBufferTransformDesc.StructureByteStride = 0;

		mDevice->CreateBuffer(&cBufferTransformDesc, NULL, &mConstantBuffer);
	}

	void InitializeCamera()
	{
		mMatrixBuffer.mProjection = mat4f::normalizedOrthographicLH(-5, 5, -5, 5, 0.1f, 100.0f).transpose();
	}

	vec3f ScreenToWorldPosition(ScreenPoint p)
	{
		return vec3f(
			(static_cast<float>(p.x) / mOptions.mWindowWidth - 0.5f) * 10,
			-(static_cast<float>(p.y) / mOptions.mWindowHeight - 0.5f) * 10,
			0
		);
	}

	vec4f mCircleScale;
	void VUpdate(double milliseconds) override
	{
		auto input = &Input::SharedInstance();

		vec4f point;
		vec3f position(0.0f, 0.0f, 0.0f);

		mCircleScale.x = 30.0f / mOptions.mWindowWidth;
		mCircleScale.y = 30.0f / mOptions.mWindowHeight;
		mCircleScale.z = 1.0f;

		auto mousePos = ScreenToWorldPosition(input->mousePosition);

		static auto pIndex = -1;
		if (input->GetMouseButtonDown(MOUSEBUTTON_LEFT))
		{
			auto minDist = 10000.0f;
			auto minIndex = -1;
			for (size_t i = 0; i < 4; i++)
			{
				auto p = vec3f(mBezier.p[i].x, mBezier.p[i].y, 0);
				auto dist = min(minDist, (mousePos - p).magnitude2());

				if (dist < minDist)
				{
					minDist = dist;
					minIndex = i;
				}
			}

			if (minDist < .025f)
			{
				pIndex = minIndex;
			}
		} else if (input->GetMouseButtonUp(MOUSEBUTTON_LEFT))
		{
			pIndex = -1;
		}

		if (pIndex > -1)
		{
			mBezier.p[pIndex].x = mousePos.x;
			mBezier.p[pIndex].y = mousePos.y;
		}

		for (size_t i = 0; i < 4; i++)
		{
			mHandlesVertices[i].mPosition.x = mBezier.p[i].x;
			mHandlesVertices[i].mPosition.y = mBezier.p[i].y;
		}

		float t;
		for (size_t i = 0, j = 0; i < BEZIER_VERTEX_COUNT; i++, j += 2)
		{
			t = static_cast<float>(i) / (BEZIER_VERTEX_COUNT - 1);
			
			mBezier.EvaluateSIMD(t, &point);
			mBezier.Evaluate(t, &point);

			mBezierVertices[i].mPosition = { point.x, point.y, point.z };
		}

		mMatrixBuffer.mWorld = mat4f::translate(position).transpose();
	}

	void VRender() override
	{
		float color[4] = { 0.2f, 0.2f, 0.2f, 1.0f };

		// Set up the input assembler
		mDeviceContext->IASetInputLayout(mInputLayout);
		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_TRIANGLE);

		mDeviceContext->RSSetViewports(1, &mRenderer->GetViewport());
		mDeviceContext->OMSetRenderTargets(1, mRenderer->GetRenderTargetView(), mRenderer->GetDepthStencilView());
		mDeviceContext->ClearRenderTargetView(*mRenderer->GetRenderTargetView(), color);
		mDeviceContext->ClearDepthStencilView(
		mRenderer->GetDepthStencilView(),
			D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL,
			1.0f,
			0);

		mDeviceContext->VSSetShader(mVertexShader, NULL, 0);
		mDeviceContext->PSSetShader(mPixelShader, NULL, 0);

		mDeviceContext->UpdateSubresource(
			mConstantBuffer,
			0,
			NULL,
			&mMatrixBuffer,
			0,
			0); 

		mDeviceContext->VSSetConstantBuffers(
			0,
			1,
			&mConstantBuffer);

		// Bezier
		mDeviceContext->UpdateSubresource(static_cast<DX11Mesh*>(mBezierMesh)->mVertexBuffer, 0, NULL, &mBezierVertices, 0, 0);

		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_LINE);
		mRenderer->VBindMesh(mBezierMesh);
		mRenderer->VDrawIndexed(0, mBezierMesh->GetIndexCount());

		// Handles
		mDeviceContext->UpdateSubresource(static_cast<DX11Mesh*>(mHandlesMesh)->mVertexBuffer, 0, NULL, &mHandlesVertices, 0, 0);

		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_LINE);
		mRenderer->VBindMesh(mHandlesMesh);
		mRenderer->VDrawIndexed(0, mHandlesMesh->GetIndexCount());

		// Circles
		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_TRIANGLE);
		for (size_t i = 0; i < 4; i++)
		{
			mMatrixBuffer.mWorld = (mat4f::scale(mCircleScale) * mat4f::translate(mBezier.p[i])).transpose();
			mDeviceContext->UpdateSubresource(mConstantBuffer, 0, NULL, &mMatrixBuffer, 0, 0);

			mRenderer->VBindMesh(mCircleMesh);
			mRenderer->VDrawIndexed(0, mCircleMesh->GetIndexCount());
		}


		mRenderer->VSwapBuffers();
	}

	void VOnResize() override
	{
		InitializeCamera();
	}

	void VShutdown() override
	{
		mBezierMesh->~IMesh();
		mAllocator.Free();
	}
};

DECLARE_MAIN(Rig3DSampleScene);