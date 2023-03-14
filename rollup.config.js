import resolve from "@rollup/plugin-node-resolve";
export default [
	{
		input: 'potree/src/Potree.js',
		treeshake: false,
		output: {
			file: 'potree/build/potree/potree.js',
			format: 'umd',
			name: 'Potree',
			sourcemap: true,
		}
	},{
		input: 'potree/src/workers/BinaryDecoderWorker.js',
		output: {
			file: 'potree/build/potree/workers/BinaryDecoderWorker.js',
			format: 'es',
			name: 'Potree',
			sourcemap: false
		}
	},{
		input: 'potree/src/modules/loader/2.0/DecoderWorker.js',
		output: {
			file: 'potree/build/potree/workers/2.0/DecoderWorker.js',
			format: 'es',
			name: 'Potree',
			sourcemap: false
		}
	},{
		input: 'potree/src/modules/loader/2.0/DecoderWorker_brotli.js',
		output: {
			file: 'potree/build/potree/workers/2.0/DecoderWorker_brotli.js',
			format: 'es',
			name: 'Potree',
			sourcemap: false
		}
	},{
		input: 'icp/app.js',
		output: [
			{
			format: 'es',
			file: 'icp/bundle.js',
			},
		],
		plugins: [resolve()],
	}
]