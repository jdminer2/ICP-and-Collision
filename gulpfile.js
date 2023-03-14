
const path = require('path');
const gulp = require('gulp');
const exec = require('child_process').exec;

const fs = require("fs");
const fsp = fs.promises;
const concat = require('gulp-concat');
const connect = require('gulp-connect');
const {watch} = gulp;

const {createExamplesPage} = require("./potree/src/tools/create_potree_page");
const {createGithubPage} = require("./potree/src/tools/create_github_page");
const {createIconsPage} = require("./potree/src/tools/create_icons_page");


let paths = {
	laslaz: [
		"potree/build/workers/laslaz-worker.js",
		"potree/build/workers/lasdecoder-worker.js",
	],
	html: [
		"potree/src/viewer/potree.css",
		"potree/src/viewer/sidebar.html",
		"potree/src/viewer/profile.html"
	],
	resources: [
		"potree/resources/**/*"
	]
};

let workers = {
	"LASLAZWorker": [
		"potree/libs/plasio/workers/laz-perf.js",
		"potree/libs/plasio/workers/laz-loader-worker.js"
	],
	"LASDecoderWorker": [
		"potree/src/workers/LASDecoderWorker.js"
	],
	"EptLaszipDecoderWorker": [
		"potree/src/workers/EptLaszipDecoderWorker.js"
	],
	"EptBinaryDecoderWorker": [
		"potree/libs/ept/ParseBuffer.js",
		"potree/src/workers/EptBinaryDecoderWorker.js"
	],
	"EptZstandardDecoderWorker": [
		"potree/src/workers/EptZstandardDecoder_preamble.js",
		'potree/libs/zstd-codec/bundle.js',
		"potree/libs/ept/ParseBuffer.js",
		"potree/src/workers/EptZstandardDecoderWorker.js"
	]
};

// these libs are lazily loaded
// in order for the lazy loader to find them, independent of the path of the html file,
// we package them together with potree
let lazyLibs = {
	"geopackage": "potree/libs/geopackage",
	"sql.js": "potree/libs/sql.js"
};

let shaders = [
	"potree/src/materials/shaders/pointcloud.vs",
	"potree/src/materials/shaders/pointcloud.fs",
	"potree/src/materials/shaders/pointcloud_sm.vs",
	"potree/src/materials/shaders/pointcloud_sm.fs",
	"potree/src/materials/shaders/normalize.vs",
	"potree/src/materials/shaders/normalize.fs",
	"potree/src/materials/shaders/normalize_and_edl.fs",
	"potree/src/materials/shaders/edl.vs",
	"potree/src/materials/shaders/edl.fs",
	"potree/src/materials/shaders/blur.vs",
	"potree/src/materials/shaders/blur.fs",
];

// For development, it is now possible to use 'gulp webserver'
// from the command line to start the server (default port is 8080)
gulp.task('webserver', gulp.series(async function() {
	server = connect.server({
		port: 1234,
		https: false,
	});
}));

gulp.task('examples_page', async function(done) {
	await Promise.all([
		createExamplesPage(),
		createGithubPage(),
	]);

	done();
});

gulp.task('icons_viewer', async function(done) {
	await createIconsPage();

	done();

});

gulp.task('test', async function() {

	console.log("asdfiae8ofh");

});

gulp.task("workers", async function(done){

	for(let workerName of Object.keys(workers)){

		gulp.src(workers[workerName])
			.pipe(concat(`${workerName}.js`))
			.pipe(gulp.dest('potree/build/potree/workers'));
	}

	done();
});

gulp.task("lazylibs", async function(done){

	for(let libname of Object.keys(lazyLibs)){

		const libpath = lazyLibs[libname];

		gulp.src([`${libpath}/**/*`])
			.pipe(gulp.dest(`potree/build/potree/lazylibs/${libname}`));
	}

	done();
});

gulp.task("shaders", async function(){

	const components = [
		"let Shaders = {};"
	];

	for(let file of shaders){
		const filename = path.basename(file);

		const content = await fsp.readFile(file);

		const prep = `Shaders["${filename}"] = \`${content}\``;

		components.push(prep);
	}

	components.push("export {Shaders};");

	const content = components.join("\n\n");

	const targetPath = `./potree/build/shaders/shaders.js`;

	if(!fs.existsSync("potree/build/shaders")){
		fs.mkdirSync("potree/build/shaders");
	}
	fs.writeFileSync(targetPath, content, {flag: "w"});
});

gulp.task('build', 
	gulp.series(
		gulp.parallel("workers", "lazylibs", "shaders", "icons_viewer", "examples_page"),
		async function(done){
			gulp.src(paths.html).pipe(gulp.dest('potree/build/potree'));

			gulp.src(paths.resources).pipe(gulp.dest('potree/build/potree/resources'));

			gulp.src(["LICENSE"]).pipe(gulp.dest('potree/build/potree'));

			done();
		}
	)
);

gulp.task("pack", async function(){
	exec('rollup -c', function (err, stdout, stderr) {
		console.log(stdout);
		console.log(stderr);
	});
});

gulp.task('watch', gulp.parallel("build", "pack", "webserver", async function() {

	let watchlist = [
		'potree/src/**/*.js',
		'potree/src/**/**/*.js',
		'potree/src/**/*.css',
		'potree/src/**/*.html',
		'potree/src/**/*.vs',
		'potree/src/**/*.fs',
		'potree/resources/**/*',
		'potree/examples//**/*.json',
		'!potree/resources/icons/index.html',
	];

	watch(watchlist, gulp.series("build", "pack"));

}));


