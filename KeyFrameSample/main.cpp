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

#define PI 3.1415926535f

#if defined(_MSC_VER)
#define ALIGNED(x) __declspec(align(x))
// other Alignment here..
#endif

using namespace Rig3D;
using namespace cliqCity::graphicsMath;

// cubic degree bezier curve
class Bezier
{
public:
	union ALIGNED(16)
	{
		struct
		{
			Vector4 b0;
			Vector4 b1;
			Vector4 b2;
			Vector4 b3;
		};
		Matrix4 m;
	};

	Bezier()
	{

	}

	~Bezier()
	{
		
	}

	void Evaluate(const float t, Vector4 result)
	{
		
	}

	void Mult(const Vector4 vec, const Matrix4 mat)
	{
		const __m128& MRow1 = _mm_load_ps(mat.u.data);

		//const __m128& xxxx = _mm_replicate_x_ps(v);
	}
};


static const int VERTEX_COUNT = 100;
static const int INDEX_COUNT = (VERTEX_COUNT - 1) * 2;
static const float ANIMATION_DURATION = 20000.0f; // 20 Seconds
static const int KEY_FRAME_COUNT = 10;

class Rig3DSampleScene : public IScene, public virtual IRendererDelegate
{
public:

	typedef cliqCity::graphicsMath::Vector2 vec2f;
	typedef cliqCity::memory::LinearAllocator LinearAllocator;

	struct SampleVertex
	{
		vec3f mPosition;
		vec3f mColor;
	};

	struct SampleMatrixBuffer
	{
		mat4f mWorld;
		mat4f mProjection;
	};

	struct KeyFrame
	{
		quatf mRotation;
		vec3f mPosition;
		float mTime;
	};

	SampleMatrixBuffer		mMatrixBuffer;
	IMesh*					mCubeMesh;
	LinearAllocator			mAllocator;
	KeyFrame				mKeyFrames[KEY_FRAME_COUNT];

	DX3D11Renderer*			mRenderer;
	ID3D11Device*			mDevice;
	ID3D11DeviceContext*	mDeviceContext;
	ID3D11Buffer*			mConstantBuffer;
	ID3D11InputLayout*		mInputLayout;
	ID3D11VertexShader*		mVertexShader;
	ID3D11PixelShader*		mPixelShader;

	float					mAnimationTime;
	bool					mIsPlaying;

	MeshLibrary<LinearAllocator> mMeshLibrary;

	Rig3DSampleScene() : mAllocator(1024)
	{
		mOptions.mWindowCaption = "Key Frame Sample";
		mOptions.mWindowWidth = 800;
		mOptions.mWindowHeight = 600;
		mOptions.mGraphicsAPI = GRAPHICS_API_DIRECTX11;
		mOptions.mFullScreen = false;
		mAnimationTime = 0.0f;
		mIsPlaying = false;
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

		InitializeAnimation();
		InitializeGeometry();
		InitializeShaders();
		InitializeCamera();
	}

	void InitializeAnimation()
	{
		std::ifstream file("Animation\\keyframe-input.txt");

		if (!file.is_open()) {
			printf("ERROR OPENING FILE");
			return;
		}

		char line[100];
		int i = 0;
		float radians = (PI / 180.f);

		while (file.good()) {
			file.getline(line, 100);
			if (line[0] == '\0') {
				continue;
			}

			float time, angle;
			vec3f position, axis;
			sscanf_s(line, "%f %f %f %f %f %f %f %f\n", &time, &position.x, &position.y, &position.z, &axis.x, &axis.y, &axis.z, &angle);
			mKeyFrames[i].mTime = time;
			mKeyFrames[i].mPosition = position;
			mKeyFrames[i].mRotation = cliqCity::graphicsMath::normalize(quatf::angleAxis(angle * radians, axis));
			i++;
		}

		file.close();

		mMatrixBuffer.mWorld = mat4f::translate(mKeyFrames[1].mPosition).transpose();
		mAnimationTime = 0.0f;
		mIsPlaying = false;
	}

	void InitializeGeometry()
	{
		SampleVertex vertices[VERTEX_COUNT];

		Bezier bezier;
		bezier.b0.x = 0;
		bezier.m.u.y = 1;
		bezier.b0.data[2] = 2;


		// allocate a sizeof VERTEX_COUNT * 2 to simplify our upcoming loop.
		uint16_t indices[VERTEX_COUNT * 2];

		auto x = -4.0f;
		auto y = -4.0f;
		auto x1 = 4.0f;
		auto y1 = 4.0f;
		auto dx = (x1 - x) / VERTEX_COUNT;
		auto dy = (y1 - y) / VERTEX_COUNT;

		for (size_t i = 0, j = 0; i < VERTEX_COUNT; i++, j += 2)
		{


			vertices[i].mPosition = { x, y, 0.0f };	// Front Top Left
			vertices[i].mColor = { 1.0f, 1.0f, 0.0f };

			indices[j] = i;
			indices[j + 1] = i + 1;

			x += dx;
			y += dy;
		}

		mMeshLibrary.NewMesh(&mCubeMesh, mRenderer);
		mRenderer->VSetMeshVertexBufferData(mCubeMesh, vertices, sizeof(SampleVertex) * VERTEX_COUNT, sizeof(SampleVertex), GPU_MEMORY_USAGE_DYNAMIC);
		mRenderer->VSetMeshIndexBufferData(mCubeMesh, indices, INDEX_COUNT, GPU_MEMORY_USAGE_STATIC);
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

		static SampleVertex vertices[VERTEX_COUNT];

		auto x = -4.0f;
		auto y = -4.0f;
		auto x1 = 4.0f;
		auto y1 = 4.0f;
		auto dx = (x1 - x) / VERTEX_COUNT;
		auto dy = (y1 - y) / VERTEX_COUNT;

		for (size_t i = 0, j = 0; i < VERTEX_COUNT; i++, j += 2)
		{
			vertices[i].mPosition = { x, y, 0.0f };	// Front Top Left
			vertices[i].mColor = { 1.0f, 1.0f, 0.0f };

			x += dx;
			y += dy;
		}

		mMatrixBuffer.mWorld = mat4f::translate(position).transpose();

		char str[256];
		sprintf_s(str, "Milliseconds %f", mAnimationTime);
		mRenderer->SetWindowCaption(str);

		if (Input::SharedInstance().GetKeyDown(KEYCODE_LEFT)) {
			InitializeAnimation();
		}
	}

	void VRender() override
	{
		float color[4] = { 0.2f, 0.2f, 0.2f, 1.0f };

		// Set up the input assembler
		mDeviceContext->IASetInputLayout(mInputLayout);
		mRenderer->VSetPrimitiveType(GPU_PRIMITIVE_TYPE_LINE);

		//mDeviceContext->RSSetViewports(1, &mRenderer->GetViewport());

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

		mRenderer->VBindMesh(mCubeMesh);

		mRenderer->VDrawIndexed(0, mCubeMesh->GetIndexCount());
		mRenderer->VSwapBuffers();
	}

	void VOnResize() override
	{
		InitializeCamera();
	}

	void VShutdown() override
	{
		mCubeMesh->~IMesh();
		mAllocator.Free();
	}
};

DECLARE_MAIN(Rig3DSampleScene);