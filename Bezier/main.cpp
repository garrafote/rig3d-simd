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

typedef ALIGNED(16) cliqCity::graphicsMath::Vector4 vec4fa;
typedef ALIGNED(16) cliqCity::graphicsMath::Matrix4 mat4fa;

using namespace Rig3D;

// cubic degree bezier curve
class ALIGNED(16) Bezier
{
public:
	vec4fa p0;
	vec4fa p1;
	vec4fa p2;
	vec4fa p3;

	Bezier() { }
	~Bezier() { }

	void Evaluate(const float time, vec4fa* result)
	{
		static mat4fa mat = mat4fa(
			 1,  0,  0,  0,
			-3,  3,  0,  0,
			 3, -6,  3,  0, 
			-1,  3, -3,  1);

		vec4fa t(1, time, time*time, time*time*time);
	}

	// Equation based on this article:
	// http://www.idav.ucdavis.edu/education/CAGDNotes/Matrix-Cubic-Bezier-Curve/Matrix-Cubic-Bezier-Curve.html
	void EvaluateSIMD(const float time, vec4fa* result)
	{
		static mat4fa m = mat4fa(
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

static const int VERTEX_COUNT = 100;
static const int INDEX_COUNT = (VERTEX_COUNT - 1) * 2;

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


	BezierMatrixBuffer		mMatrixBuffer;
	IMesh*					mBezierMesh;
	LinearAllocator			mAllocator;

	DX3D11Renderer*			mRenderer;
	ID3D11Device*			mDevice;
	ID3D11DeviceContext*	mDeviceContext;
	ID3D11Buffer*			mConstantBuffer;
	ID3D11InputLayout*		mInputLayout;
	ID3D11VertexShader*		mVertexShader;
	ID3D11PixelShader*		mPixelShader;

	MeshLibrary<LinearAllocator> mMeshLibrary;

	Rig3DSampleScene() : mAllocator(1024)
	{
		mOptions.mWindowCaption = "SIMD Bezier";
		mOptions.mWindowWidth = 800;
		mOptions.mWindowHeight = 600;
		mOptions.mGraphicsAPI = GRAPHICS_API_DIRECTX11;
		mOptions.mFullScreen = false;
		mMeshLibrary.SetAllocator(&mAllocator);
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

		VOnResize();

		InitializeGeometry();
		InitializeShaders();
		InitializeCamera();
	}

	void InitializeGeometry()
	{
		BezierVertex vertices[VERTEX_COUNT];

		Bezier bezier;
		//bezier.p0 = vec4fa(-4, -4, 0, 0);
		//bezier.p2 = vec4fa(-8,  4, 0, 0);
		//bezier.p3 = vec4fa( 4, -4, 0, 0);
		//bezier.p1 = vec4fa( 8,  4, 0, 0);

		bezier.p0 = vec4fa(-4, -4, 0, 0);
		bezier.p1 = vec4fa(-4,  4, 0, 0);
		bezier.p2 = vec4fa( 4, -4, 0, 0);
		bezier.p3 = vec4fa( 4,  4, 0, 0);


		vec4fa point;

		// allocate a sizeof VERTEX_COUNT * 2 to simplify our upcoming loop.
		uint16_t indices[VERTEX_COUNT * 2];

		float t;
		for (size_t i = 0, j = 0; i < VERTEX_COUNT; i++, j += 2)
		{
			t = (float)i / (VERTEX_COUNT - 1);
			bezier.EvaluateSIMD(t, &point);

			vertices[i].mPosition = { point.x, point.y, 0.0f };	// Front Top Left
			vertices[i].mColor = { 1.0f, 1.0f, 0.0f };

			indices[j] = i;
			indices[j + 1] = i + 1;
		}

		mMeshLibrary.NewMesh(&mBezierMesh, mRenderer);
		mRenderer->VSetMeshVertexBufferData(mBezierMesh, vertices, sizeof(BezierVertex) * VERTEX_COUNT, sizeof(BezierVertex), GPU_MEMORY_USAGE_DYNAMIC);
		mRenderer->VSetMeshIndexBufferData(mBezierMesh, indices, INDEX_COUNT, GPU_MEMORY_USAGE_STATIC);
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

	void VUpdate(double milliseconds) override
	{
		vec3f position(0.0f, 0.0f, 0.0f);

		BezierVertex vertices[VERTEX_COUNT];

		Bezier bezier;
		bezier.p0 = vec4fa(-4, -4, 0, 0);
		bezier.p1 = vec4fa(-4, 4, 0, 0);
		bezier.p2 = vec4fa(4, -4, 0, 0);
		bezier.p3 = vec4fa(4, 4, 0, 0);

		vec4fa point;

		// allocate a sizeof VERTEX_COUNT * 2 to simplify our upcoming loop.
		uint16_t indices[VERTEX_COUNT * 2];

		float t;
		for (size_t i = 0, j = 0; i < VERTEX_COUNT; i++, j += 2)
		{
			t = (float)i / (VERTEX_COUNT - 1);
			bezier.EvaluateSIMD(t, &point);

			vertices[i].mPosition = { point.x, point.y, point.z }; // Front Top Left
			vertices[i].mColor = { 1.0f, 1.0f, 0.0f };

			indices[j] = i;
			indices[j + 1] = i + 1;
		}



		mMatrixBuffer.mWorld = mat4f::translate(position).transpose();
	}

	void VRender() override
	{
		float color[4] = { 0.2f, 0.2f, 0.2f, 1.0f };

		// Set up the input assembler
		mDeviceContext->IASetInputLayout(mInputLayout);
		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_LINE);

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

		mRenderer->VBindMesh(mBezierMesh);

		mRenderer->VDrawIndexed(0, mBezierMesh->GetIndexCount());
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