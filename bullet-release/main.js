const fs = require('fs');
const path = require('path');

function search_upwards(pth, operation) {
    let currentDir = pth;
    if (fs.statSync(pth).isFile()) {
        currentDir = path.dirname(pth);
    } else if (fs.statSync(pth).isDirectory()) {
        currentDir = pth;
    } else {
        return '/';
    }

    while (currentDir !== '/') {
        if (operation(currentDir)) {
            return currentDir;
        }
        currentDir = path.dirname(currentDir);
    }
    return '/';
}

function is_engine_path(pth) {
    const currentDir = pth;
    const pkgJsonPath = path.join(currentDir, 'package.json');
    if (fs.existsSync(pkgJsonPath)) {
        const pkgJson = JSON.parse(fs.readFileSync(pkgJsonPath, 'utf8'));
        if (pkgJson.name === 'cocos') {
            return true;
        }
    }
    return false;
}

function is_project_path(pth) {
    const currentDir = pth;
    const ccDtsPath = path.join(currentDir, 'temp', 'declarations', 'cc.d.ts');
    if (fs.existsSync(ccDtsPath)) {
        return true;
    }
    return false;
}

var myPath = 'd:\\'

var res = search_upwards(myPath, is_engine_path);

if (res !== '') {
	console.log(`found at ${res}`);
} else {
	console.log(`not found`);
}
